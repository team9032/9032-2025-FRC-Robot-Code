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

        previousSpeed = swerve.getLocalization().getCurrentSpeed();
        previousTime = Utils.getCurrentTimeSeconds();
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getLocalization().getCurrentPose();

        var driveDirection = path.getPathDirection(currentPose.getTranslation());
        double remaingPathDistance = path.getRemainingPathDistance(currentPose.getTranslation());

        /* Use kinematics to find the maximum drive speed that can be stopped with the max acceleration */
        double driveSpeed = Math.sqrt(Math.pow(path.endingSpeed(), 2) - (2.0 * kMaxAcceleration * -remaingPathDistance));

        /* Apply speed limit */
        driveSpeed = Math.min(driveSpeed, kMaxSpeed);

        /* Find acceleration */
        double currentTime = Utils.getCurrentTimeSeconds();
        double dt = currentTime - previousTime;
        double acceleration = (driveSpeed - previousSpeed) / dt;

        double maxForwardAcceleration;
        /* If we are in the torque limited part of the motor curve, limit forward acceleration based on available torque */
        if (driveSpeed >= kTorqueLimitedSpeedStart) {
            double currentSpeed = swerve.getLocalization().getCurrentSpeed();

            maxForwardAcceleration = kMaxAcceleration * (1.0 - (currentSpeed / kTrueMaxSpeed));
        }

        /* Use the true max acceleration since we are in the current limited part of the motor curves */
        else 
            maxForwardAcceleration = kMaxAcceleration;

        /* Apply forward and reverse acceleration limit */
        acceleration = MathUtil.clamp(acceleration, -kMaxAcceleration, maxForwardAcceleration);
        driveSpeed = previousSpeed + (acceleration * dt);

        /* Find x and y velocities */
        double xVelocity = driveSpeed * driveDirection.getX();
        double yVelocity = driveSpeed * driveDirection.getY();

        /* Find angular velocity by combining the profiled PID's output and its velocity setpoint */
        double angularVelocity = alignmentRotationPID.calculate(currentPose.getRotation().getRadians()) + alignmentRotationPID.getSetpoint().velocity;

        swerve.setControl(
            kFieldCentricClosedLoopDriveRequest
                .withVelocityX(xVelocity)
                .withVelocityY(yVelocity)
                .withRotationalRate(angularVelocity)
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

    private boolean atSetpoint() {
        var currentTranslation = swerve.getLocalization().getCurrentPose().getTranslation();
        double currentSpeed = swerve.getLocalization().getCurrentSpeed();

        return alignmentRotationPID.atGoal() 
            && currentTranslation.getDistance(path.finalPose().getTranslation()) < kXYAlignmentTolerance.in(Meters)
            && currentSpeed < kAcceptableEndingSpeed;
    }

    @Override
    public boolean isFinished() {
        if (path.endingSpeed() > 0.0) {
            var currentTranslation = swerve.getLocalization().getCurrentPose().getTranslation();

            return currentTranslation.getDistance(path.finalPose().getTranslation()) < kXYAlignmentTolerance.in(Meters);//TODO have a rougher tolerance here
        }

        else 
            return atSetpoint();
    }
}