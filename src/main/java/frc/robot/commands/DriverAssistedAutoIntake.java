package frc.robot.commands;

import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.localization.Localization;
import frc.robot.localization.TrackedObject;
import frc.robot.subsystems.swerve.KrakenSwerve;

import static frc.robot.Constants.ObjectAimingConstants.*;
import static frc.robot.Constants.PathplannerConfig.kClosedLoopDriveRequest;

public class DriverAssistedAutoIntake extends Command {
    private final KrakenSwerve swerve;
    private final DoubleSupplier xSpeedSupplier;
    private final DoubleSupplier ySpeedSupplier;
    private final Localization localization;

    private final PIDController rotationController;

    private TrackedObject lastCoralTarget;

    public DriverAssistedAutoIntake(DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier, KrakenSwerve swerve) {
        this.swerve = swerve;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        
        localization = swerve.getLocalization();

        rotationController = new PIDController(kPRotation, 0, kDRotation);
        rotationController.enableContinuousInput(-180.0, 180.0);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        double currentYaw = swerve.drivetrain.getState().Pose.getRotation().getDegrees();

        /* Prevent rotating on init if no target is seen */
        rotationController.setSetpoint(currentYaw);
    }

    @Override
    public void execute() {
        double currentYaw = swerve.drivetrain.getState().Pose.getRotation().getDegrees();

        var coralTarget = getCoralTarget();

        /* Update setpoint if we have a target */
        if (coralTarget != null) {
            double rotationSetpoint = coralTarget.objectYawInFieldSpace() + kRotationSetpoint;

            rotationController.setSetpoint(MathUtil.inputModulus(rotationSetpoint, -180.0, 180.0));
        }

        double magnitude = Math.sqrt(Math.pow(xSpeedSupplier.getAsDouble(), 2) + Math.pow(ySpeedSupplier.getAsDouble(), 2));

        /* Drive based on desired speed and the rotation PID's output */
        var speeds = new ChassisSpeeds(
            magnitude * kMaxDrivingSpeed,
            0.0,
            rotationController.calculate(currentYaw)
        );

        swerve.drivetrain.setControl(kClosedLoopDriveRequest.withSpeeds(speeds));
    }

    private TrackedObject getCoralTarget() {
        TrackedObject coralTarget;

        var coralTargets = localization.getTrackedObjectsFromCamera(kObjectTrackingCameraName);
        
        /* Remove targets that are not coral  */
        coralTargets.removeIf((target) -> !target.isCoral());

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
                double yawDifference = Math.abs(target.objectYawInFieldSpace() - lastCoralTarget.objectYawInFieldSpace());

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
            if(target.objectPitchInCameraSpace() < closetCoral.objectPitchInCameraSpace()) {
                closetCoral = target;
            } 
        }

        return closetCoral;
    }

    @Override
    public void end(boolean interrupted) {
        rotationController.reset();

        lastCoralTarget = null;
    }
}