package frc.robot.commands;

import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.localization.Localization;
import frc.robot.localization.TrackedObject;
import frc.robot.localization.TrackedObject.ObjectType;
import frc.robot.subsystems.swerve.KrakenSwerve;

import static frc.robot.Constants.DriverAssistConstants.*;
import static frc.robot.Constants.DriverConstants.kMaxSpeed;
import static frc.robot.Constants.DriverConstants.kRotationRate;
import static frc.robot.Constants.ObjectAimingConstants.kObjectTrackingCameraName;
import static frc.robot.Constants.PathplannerConfig.kRobotRelativeClosedLoopDriveRequest;

public class IntakeDriverAssist extends Command {
    private final KrakenSwerve swerve;

    private final DoubleSupplier rotSup;
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;

    private final Localization localization;

    private TrackedObject lastCoralTarget;

    public IntakeDriverAssist(KrakenSwerve swerve, DoubleSupplier rotSup, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
        this.swerve = swerve;

        this.rotSup = rotSup;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        
        localization = swerve.getLocalization();

        addRequirements(swerve);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        var coralTarget = getCoralTarget();

        /* Gets the robot relative speeds as if we are driving normally */
        var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            translationSup.getAsDouble() * kMaxSpeed, 
            strafeSup.getAsDouble() * kMaxSpeed,
            rotSup.getAsDouble() * kRotationRate,
            localization.getCurrentPose().getRotation()
                .minus(swerve.getOperatorPerspective())//Apply the operator perspective
        );

        if (coralTarget != null) {
            double magnitude = Math.sqrt(Math.pow(translationSup.getAsDouble(), 2) + Math.pow(strafeSup.getAsDouble(), 2));

            speeds.vyMetersPerSecond = MathUtil.clamp(-coralTarget.getPhotonVisionData().yaw * kPTranslation, -magnitude, magnitude);
        }

        else if (lastCoralTarget != null) {
            double magnitude = Math.sqrt(Math.pow(translationSup.getAsDouble(), 2) + Math.pow(strafeSup.getAsDouble(), 2));

            speeds.vyMetersPerSecond = MathUtil.clamp(-lastCoralTarget.getPhotonVisionData().yaw * kPTranslation, -magnitude, magnitude);
        }

        swerve.setControl(kRobotRelativeClosedLoopDriveRequest.withSpeeds(speeds));
    }

    private TrackedObject getCoralTarget() {
        TrackedObject coralTarget;

        var coralTargets = localization.getTrackedObjectsFromCameraWithType(kObjectTrackingCameraName, ObjectType.CORAL);
        /* Exit if no targets exist */
        if (coralTargets.isEmpty())
            return null;

        /* Find the target that is closest if there is no last target */
        if(lastCoralTarget == null)
            coralTarget = getClosestCoral(coralTargets);

        /* Find the lowest yaw difference compared to the last coral target */
        else {            
            double lowestYawDifference = Double.MAX_VALUE;

            coralTarget = getClosestCoral(coralTargets);

            for(TrackedObject target : coralTargets) {
                double yawDifference = Math.abs(target.getPhotonVisionData().getYaw() - lastCoralTarget.getPhotonVisionData().getYaw());

                if(yawDifference < lowestYawDifference) {
                    lowestYawDifference = yawDifference;

                    coralTarget = target;
                }
            }
        }

        lastCoralTarget = coralTarget;        

        return coralTarget;
    }

    private TrackedObject getClosestCoral(List<TrackedObject> coralTargets) {
        TrackedObject closetCoral = coralTargets.get(0);

        for(var target : coralTargets) {
            if(target.getPhotonVisionData().getPitch() < closetCoral.getPhotonVisionData().getPitch()) {
                closetCoral = target;
            } 
        }

        return closetCoral;
    }

    @Override
    public void end(boolean interrupted) {
        lastCoralTarget = null;
    }
}