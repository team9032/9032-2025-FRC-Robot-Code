package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.pathing.CurvedPath;
import frc.robot.subsystems.swerve.KrakenSwerve;

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

        var driveDirection = path.getPathDirection(currentPose);
        double remaingPathDistance = path.getRemainingPathDistance(currentPose);

        double profiledDriveSpeed = Math.sqrt(0.0 - (2.0 * kMaxAcceleration * -remaingPathDistance));//TODO final velocity
        profiledDriveSpeed = Math.min(profiledDriveSpeed, kMaxSpeed);

        double currentTime = Utils.getCurrentTimeSeconds();
        double dt = currentTime - previousTime;
        double acceleration = (profiledDriveSpeed - previousSpeed) / dt;

        double maxAcceleration = kMaxAcceleration;
        if (profiledDriveSpeed > kTorqueLimitedSpeedStart) {
            //TODO implement
        }

        acceleration = MathUtil.clamp(acceleration, -maxAcceleration, maxAcceleration);
        profiledDriveSpeed = previousSpeed + (acceleration * dt);

        double xVelocity = profiledDriveSpeed * driveDirection.getX();
        double yVelocity = profiledDriveSpeed * driveDirection.getY();

        double rot = alignmentRotationPID.calculate(currentPose.getRotation().getRadians()) + alignmentRotationPID.getSetpoint().velocity;

        swerve.setControl(
            kFieldCentricClosedLoopDriveRequest
                .withVelocityX(xVelocity)
                .withVelocityY(yVelocity)
                .withRotationalRate(rot)
        );

        previousSpeed = profiledDriveSpeed;
        previousTime = currentTime;

        SmartDashboard.putBoolean("Simple Drive To Pose At Goal", atGoal());
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(kFieldCentricClosedLoopDriveRequest
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(0.0)
        );
    }

    private boolean atGoal() {
        return alignmentRotationPID.atGoal();
    }

    @Override
    public boolean isFinished() {
        return false; //atGoal();
    }
}