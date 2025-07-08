package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.localization.Localization;
import frc.robot.localization.TrackedObject;
import frc.robot.localization.TrackedObject.ObjectType;
import frc.robot.subsystems.swerve.KrakenSwerve;

import static frc.robot.Constants.IntakeDriverAssistConstants.*;
import static frc.robot.Constants.PathFollowingConstants.kMaxDrivingSpeed;
import static frc.robot.Constants.PathFollowingConstants.kRobotRelativeClosedLoopDriveRequest;
import static frc.robot.Constants.PathFollowingConstants.kSlowDistanceToCoral;
import static frc.robot.Constants.PathFollowingConstants.kSlowDrivingSpeed;

public class RotationalDriveToCoral extends Command {
    private final KrakenSwerve swerve;
    private final Localization localization;

    private final PIDController rotationController;

    private TrackedObject lastCoralTarget;

    public RotationalDriveToCoral(KrakenSwerve swerve) {
        this.swerve = swerve;
        
        localization = swerve.getLocalization();

        rotationController = new PIDController(kPRotationToObject, 0, kDRotationToObject);
        rotationController.enableContinuousInput(-180.0, 180.0);

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
        double currentYaw = swerve.getLocalization().getCurrentPose().getRotation().getDegrees();

        var coralTarget = getCoralTarget();

        /* Update setpoint if we have a target */
        if (coralTarget.isPresent()) {
            lastCoralTarget = coralTarget.get();

            double rotationSetpoint = currentYaw - (coralTarget.get().getPhotonVisionData().yaw - kRotationSetpoint);

            rotationController.setSetpoint(MathUtil.inputModulus(rotationSetpoint, -180.0, 180.0));
        }

        double xSpeed = kMaxDrivingSpeed;
        if (lastCoralTarget != null) {
            double distanceToCoral = lastCoralTarget.getFieldPosition().getTranslation().getDistance(swerve.getLocalization().getCurrentPose().getTranslation());

            if (distanceToCoral < kSlowDistanceToCoral) 
                xSpeed = kSlowDrivingSpeed;//TODO prevent hitting wall
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
    public void end(boolean interrupted) {
        swerve.setControl(kRobotRelativeClosedLoopDriveRequest.withSpeeds(new ChassisSpeeds()));

        rotationController.reset();

        lastCoralTarget = null;
    }
}