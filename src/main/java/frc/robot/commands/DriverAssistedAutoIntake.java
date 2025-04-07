package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.localization.Localization;
import frc.robot.subsystems.swerve.KrakenSwerve;

import static frc.robot.Constants.ObjectAimingConstants.*;
import static frc.robot.Constants.PathplannerConfig.kClosedLoopDriveRequest;

public class DriverAssistedAutoIntake extends Command {
    private final KrakenSwerve swerve;
    private final DoubleSupplier xSpeedSupplier;
    private final Localization localization;

    private final PIDController rotationController;

    private PhotonTrackedTarget lastCoralTarget;

    public DriverAssistedAutoIntake(DoubleSupplier xSpeedSupplier, KrakenSwerve swerve) {
        this.swerve = swerve;
        this.xSpeedSupplier = xSpeedSupplier;
        
        localization = swerve.getLocalization();

        rotationController = new PIDController(kPRotation, 0, kDRotation);
        rotationController.enableContinuousInput(-180.0, 180.0);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double currentYaw = swerve.drivetrain.getState().Pose.getRotation().getDegrees();

        var coralTarget = getCoralTarget();

        /* Update setpoint if we have a target */
        if (coralTarget != null) {
            double rotationSetpoint = currentYaw - (coralTarget.yaw - kRotationSetpoint);

            rotationController.setSetpoint(MathUtil.inputModulus(rotationSetpoint, -180.0, 180.0));
        }

        /* Drive based on desired speed and the rotation PID's output */
        var speeds = new ChassisSpeeds(
            xSpeedSupplier.getAsDouble(),
            0.0,
            rotationController.calculate(currentYaw)
        );

        swerve.drivetrain.setControl(kClosedLoopDriveRequest.withSpeeds(speeds));
    }

    private PhotonTrackedTarget getCoralTarget() {
        PhotonTrackedTarget coralTarget;

        var result = getLatestPipelineResult();

        /* Remove targets that are not coral  */
        var coralTargets = result.getTargets();
        coralTargets.removeIf((target) -> target.getDetectedObjectClassID() != kCoralId);

        /* Find the target that is closest if there is no last target */
        if(lastCoralTarget == null)
            coralTarget = getClosestCoral(coralTargets);

        /* Find the lowest yaw difference compared to the last coral target */
        else {            
            double lowestYawDifference = Double.MAX_VALUE;

            coralTarget = getClosestCoral(coralTargets);

            for(PhotonTrackedTarget target : coralTargets) {
                double yawDifference = Math.abs(target.getYaw() - lastCoralTarget.getYaw());

                if(yawDifference < lowestYawDifference) {
                    lowestYawDifference = yawDifference;

                    coralTarget = target;
                }
            }
        }

        lastCoralTarget = coralTarget;        

        return coralTarget;
    }

    private PhotonTrackedTarget getClosestCoral(List<PhotonTrackedTarget> coralTargets) {
        PhotonTrackedTarget closetCoral = coralTargets.get(0);

        for(var target : coralTargets) {
            if(target.getPitch() < closetCoral.getPitch()) {
                closetCoral = target;
            } 
        }

        return closetCoral;
    }

    private PhotonPipelineResult getLatestPipelineResult() {
        var results = new ArrayList<>(localization.getObjectTrackingResults(kObjectTrackingCameraName));

        /* Remove all results without targets */
        results.removeIf((result) -> !result.hasTargets());

        /* Find the most recent result */
        PhotonPipelineResult latestResult = results.get(0);
        for(var result : results) {
            if(result.getTimestampSeconds() > latestResult.getTimestampSeconds())
                latestResult = result;
        }

        return latestResult;        
    }

    @Override
    public void end(boolean interrupted) {
        rotationController.reset();

        lastCoralTarget = null;
    }
}