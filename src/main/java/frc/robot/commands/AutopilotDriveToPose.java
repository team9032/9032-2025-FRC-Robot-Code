package frc.robot.commands;

import static frc.robot.Constants.PathFollowingConstants.*;

import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.KrakenSwerve;

public class AutopilotDriveToPose extends Command {
    private final KrakenSwerve swerve;

    private final APTarget target;
    private final Autopilot autopilotController;

    private final ProfiledPIDController rotationController;

    public AutopilotDriveToPose(KrakenSwerve swerve, Pose2d targetPose, Rotation2d entryAngle) {
        this(swerve, targetPose, entryAngle, 0.0);
    }

    public AutopilotDriveToPose(KrakenSwerve swerve, Pose2d targetPose, Rotation2d entryAngle, double endingVelocity) {
        this.swerve = swerve;

        target = new APTarget(targetPose)
            .withEntryAngle(entryAngle)
            .withVelocity(endingVelocity);

        var profile = new APProfile(kAPConstraints)
            .withErrorXY(kXYAlignmentTolerance)
            .withErrorTheta(kRotAlignmentTolerance)
            .withBeelineRadius(kBeelineRadius);

        autopilotController = new Autopilot(profile);

        rotationController = new ProfiledPIDController(kAlignmentRotkP, 0, kAlignmentRotkD, kDriveToPoseRotationConstraints);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerve);
    }

    public AutopilotDriveToPose enterAtTargetAngle(KrakenSwerve swerve, Pose2d targetPose) {
        return new AutopilotDriveToPose(swerve, targetPose, targetPose.getRotation().unaryMinus());
    }

    @Override
    public void initialize() {
        rotationController.reset(swerve.getLocalization().getCurrentPose().getRotation().getRadians());
    }

    @Override
    public void execute() {
        var currentPose = swerve.getLocalization().getCurrentPose();
        
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            swerve.getLocalization().getCurrentVelocity(), 
            currentPose.getRotation()
        );
        Translation2d velocityTranslation = new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);

        APResult result = autopilotController.calculate(currentPose, velocityTranslation, target);
        
        rotationController.setGoal(result.targetAngle().getRadians());
        double angularVelocity = rotationController.calculate(currentPose.getRotation().getRadians()) + rotationController.getSetpoint().velocity;

        swerve.setControl(
            kFieldCentricClosedLoopDriveRequest
                .withVelocityX(result.vx())
                .withVelocityY(result.vy())
                .withRotationalRate(angularVelocity)
        );
    }

    @Override
    public void end(boolean interrupted) {
        if (target.getVelocity() == 0.0)
            swerve.setControl(kRobotRelativeClosedLoopDriveRequest.withSpeeds(new ChassisSpeeds())); 
    }

    @Override
    public boolean isFinished() {
        return autopilotController.atTarget(swerve.getLocalization().getCurrentPose(), target);
    }
}
