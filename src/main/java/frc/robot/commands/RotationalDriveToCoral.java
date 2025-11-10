package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.localization.Localization;
import frc.robot.localization.TrackedObject;
import frc.robot.localization.TrackedObject.ObjectType;
import frc.robot.subsystems.swerve.KrakenSwerve;

import static frc.robot.Constants.IntakeDriverAssistConstants.*;
import static frc.robot.Constants.PathFollowingConstants.kEndTime;
import static frc.robot.Constants.PathFollowingConstants.kMaxDrivingSpeed;
import static frc.robot.Constants.PathFollowingConstants.kRobotRelativeClosedLoopDriveRequest;
import static frc.robot.Constants.PathFollowingConstants.kSlowDistanceToCoral;
import static frc.robot.Constants.PathFollowingConstants.kSlowDrivingSpeed;

public class RotationalDriveToCoral extends Command {
    private final KrakenSwerve swerve;
    private final Localization localization;

    private final PIDController rotationController;

    private TrackedObject lastCoralTarget;

    private boolean shouldDriveSlow = false;

    private final Timer endTimer;

    public RotationalDriveToCoral(KrakenSwerve swerve) {
        this.swerve = swerve;
        
        localization = swerve.getLocalization();

        rotationController = new PIDController(kPRotationToObject, 0, kDRotationToObject);
        rotationController.enableContinuousInput(-180.0, 180.0);

        endTimer = new Timer();

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        double currentYaw = swerve.getLocalization().getCurrentPose().getRotation().getDegrees();

        /* Prevent rotating on init if no target is seen */
        rotationController.setSetpoint(currentYaw);
    }

    @Override
    public void execute() {
        var currentPose = swerve.getLocalization().getCurrentPose();
        double currentYaw = currentPose.getRotation().getDegrees();

        var coralTarget = getCoralTarget();

        /* Update setpoint if we have a target and we are not close to ending */
        if (coralTarget.isPresent() && !endTimer.isRunning()) {
            lastCoralTarget = coralTarget.get();

            /* Find the angle that points the robot towards the coral */
            double rotationSetpoint = currentPose.getTranslation().minus(lastCoralTarget.getFieldPosition().getTranslation())
                .getAngle().plus(Rotation2d.k180deg).getDegrees();
            /* Apply the rotation setpoint in robot space */
            rotationSetpoint += kRotationSetpoint;//TODO test this

            rotationController.setSetpoint(MathUtil.inputModulus(rotationSetpoint, -180.0, 180.0));
        }

        double xSpeed = 0.0;//Default to not driving if coral was never seen

        if (lastCoralTarget != null) {//TODO prevent hitting wall
            double distanceToCoral = lastCoralTarget.getFieldPosition().getTranslation().getDistance(swerve.getLocalization().getCurrentPose().getTranslation());

            if (distanceToCoral < kSlowDistanceToCoral) 
                shouldDriveSlow = true;//Persist slow mode

            if (distanceToCoral < kEndDistanceToCoral)
                endTimer.start();

            xSpeed = shouldDriveSlow ? kSlowDrivingSpeed : kMaxDrivingSpeed;
        }

        /* Gets the robot relative speeds as if we are driving normally */
        var speeds = new ChassisSpeeds(
            xSpeed, 
            0.0,
            rotationController.calculate(currentYaw)//Apply the rotation PID's output
        );

        swerve.setControl(kRobotRelativeClosedLoopDriveRequest.withSpeeds(speeds));
    }

    private Optional<TrackedObject> getCoralTarget() {
        /* Persistently track coral targets */
        if (lastCoralTarget != null) {
            var optionalCoral = localization.getTrackedObjectWithTrackingID(lastCoralTarget.getTrackingId());

            if (optionalCoral.isPresent()) 
                return optionalCoral;
        }

        return localization.getNearestObjectOfType(ObjectType.CORAL);
    }

    @Override
    public boolean isFinished() {
        return endTimer.isRunning() && endTimer.hasElapsed(kEndTime);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(kRobotRelativeClosedLoopDriveRequest.withSpeeds(new ChassisSpeeds()));

        rotationController.reset();

        lastCoralTarget = null;

        shouldDriveSlow = false;

        endTimer.stop();
        endTimer.reset();
    }
}