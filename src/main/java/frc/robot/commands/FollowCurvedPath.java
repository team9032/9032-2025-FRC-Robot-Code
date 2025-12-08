package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.pathing.CurvedPath;
import frc.robot.subsystems.swerve.KrakenSwerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import static frc.robot.pathing.PathingConstants.*;

import com.ctre.phoenix6.Utils;

import static frc.robot.Constants.PathFollowingConstants.kFieldCentricClosedLoopDriveRequest;

public class FollowCurvedPath extends Command {
    private final ProfiledPIDController alignmentRotationPID;

    private final KrakenSwerve swerve;

    private final CurvedPath path;
    
    private double previousSpeed;
    private double previousTime;

    public FollowCurvedPath(KrakenSwerve swerve, CurvedPath path) {
        alignmentRotationPID = new ProfiledPIDController(kRotationkP, 0, kRotationkD, kRotationConstraints);
        alignmentRotationPID.setTolerance(kRotAlignmentTolerance.in(Radians));
        alignmentRotationPID.enableContinuousInput(-Math.PI, Math.PI);

        this.swerve = swerve;

        this.path = path;      

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = swerve.getLocalization().getCurrentPose();
        ChassisSpeeds currentVelocity = swerve.getLocalization().getCurrentVelocity();

        alignmentRotationPID.setGoal(path.finalPose().getRotation().getRadians());
        alignmentRotationPID.reset(currentPose.getRotation().getRadians(), currentVelocity.omegaRadiansPerSecond);

        previousSpeed = Math.hypot(currentVelocity.vxMetersPerSecond, currentVelocity.vyMetersPerSecond);
        previousTime = Utils.getCurrentTimeSeconds();
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getLocalization().getCurrentPose();

        var driveDirection = path.getPathDirection(currentPose.getTranslation());
        double remaingPathDistance = path.getRemainingPathDistance(currentPose.getTranslation());

        /* Use kinematics to find the maximum drive speed that can be stopped with the max acceleration */
        double driveSpeed = Math.sqrt(0.0 - (2.0 * kMaxAcceleration * -remaingPathDistance));//TODO final velocity

        /* Apply speed limit */
        driveSpeed = Math.min(driveSpeed, kMaxSpeed);

        /* Find acceleration */
        double currentTime = Utils.getCurrentTimeSeconds();
        double dt = currentTime - previousTime;
        double acceleration = (driveSpeed - previousSpeed) / dt;

        double maxAcceleration = kMaxAcceleration;
        if (driveSpeed > kTorqueLimitedSpeedStart) {
            //TODO implement
        }

        /* Apply acceleration limit */
        acceleration = MathUtil.clamp(acceleration, -maxAcceleration, maxAcceleration);
        driveSpeed = previousSpeed + (acceleration * dt);

        /* Find x and y velocities */
        double xVelocity = driveSpeed * driveDirection.getX();
        double yVelocity = driveSpeed * driveDirection.getY();

        double rot = alignmentRotationPID.calculate(currentPose.getRotation().getRadians()) + alignmentRotationPID.getSetpoint().velocity;

        swerve.setControl(
            kFieldCentricClosedLoopDriveRequest
                .withVelocityX(xVelocity)
                .withVelocityY(yVelocity)
                .withRotationalRate(rot)
        );

        previousSpeed = driveSpeed;
        previousTime = currentTime;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(kFieldCentricClosedLoopDriveRequest
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(0.0)
        );
    }

    private boolean shouldFinish() {
        var currentTranslation = swerve.getLocalization().getCurrentPose().getTranslation();
        ChassisSpeeds currentVelocity = swerve.getLocalization().getCurrentVelocity();

        double currentSpeed = Math.hypot(currentVelocity.vxMetersPerSecond, currentVelocity.vyMetersPerSecond);

        return alignmentRotationPID.atGoal() 
            && currentTranslation.getDistance(path.finalPose().getTranslation()) < kXYAlignmentTolerance.in(Meters)
            && currentSpeed < kAcceptableEndingSpeed;
    }

    @Override
    public boolean isFinished() {
        return shouldFinish();
    }
}