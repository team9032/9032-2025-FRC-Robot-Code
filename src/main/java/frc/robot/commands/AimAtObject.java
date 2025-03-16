package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.localization.Localization;
import frc.robot.subsystems.swerve.KrakenSwerve;
import frc.robot.utils.ElasticUtil;
import frc.robot.utils.VisionTargetCache;

import static frc.robot.Constants.ObjectAimingConstants.*;
import static frc.robot.Constants.PathplannerConfig.kClosedLoopDriveRequest;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AimAtObject extends Command {
    private final KrakenSwerve swerve;
    
    private final Localization localization;

    private final VisionTargetCache<PhotonTrackedTarget> targetCache;
    
    private final PIDController rotationController;

    private final int objectToTrackId;    

    private final DoubleSupplier obstacleDistanceSup;

    private final boolean moveOnInit;

    public AimAtObject(KrakenSwerve swerve, int objectToTrackId, DoubleSupplier obstacleDistanceSup, boolean moveOnInit) {
       this.swerve = swerve; 
       this.objectToTrackId = objectToTrackId;
       this.obstacleDistanceSup = obstacleDistanceSup;
       this.moveOnInit = moveOnInit;

       localization = swerve.getLocalization();

       rotationController = new PIDController(kPRotation, 0.0, kDRotation);
       rotationController.enableContinuousInput(-180.0, 180.0);

       targetCache = new VisionTargetCache<>(kCycleAmtSinceTargetSeenCutoff);

       addRequirements(swerve);
    }

    @Override
    public void initialize() {
        ElasticUtil.sendInfo("Started object aiming");

        if (moveOnInit)
            swerve.drivetrain.setControl(kClosedLoopDriveRequest.withSpeeds(new ChassisSpeeds(kMaxDrivingSpeed, 0.0, 0.0)));

        else 
            swerve.drivetrain.setControl(kClosedLoopDriveRequest.withSpeeds(new ChassisSpeeds()));
    }  

    @Override
    public void execute() {
        if (targetCache.targetExpired()) {
            targetCache.reset();

            swerve.drivetrain.setControl(kClosedLoopDriveRequest.withSpeeds(new ChassisSpeeds()));
        }

        double currentYaw = swerve.drivetrain.getState().Pose.getRotation().getDegrees();

        PhotonTrackedTarget targetToTrack = getTarget();

        /* Wait to get a target */
        if (!targetCache.hasTarget()) {
            if (moveOnInit) {
                double drivingSpeed = obstacleDistanceSup.getAsDouble() < kSlowObstacleDistance ?
                    kSlowDrivingSpeed : kMaxDrivingSpeed;

                swerve.drivetrain.setControl(kClosedLoopDriveRequest.withSpeeds(new ChassisSpeeds(drivingSpeed, 0.0, 0.0)));
            }

            return;
        }

        /* Update setpoint if we have a target */
        if(targetToTrack != null) {
            double rotationSetpoint = currentYaw - (targetToTrack.yaw - kRotationSetpoint);

            rotationController.setSetpoint(MathUtil.inputModulus(rotationSetpoint, -180.0, 180.0));
        }

        /* Drive based on target yaw and obstacle distance */
        double drivingSpeed = obstacleDistanceSup.getAsDouble() < kSlowObstacleDistance ?
            kSlowDrivingSpeed : kMaxDrivingSpeed;

        var speeds = new ChassisSpeeds(
            drivingSpeed,
            0.0, 
            rotationController.calculate(currentYaw)
        );

        swerve.drivetrain.setControl(kClosedLoopDriveRequest.withSpeeds(speeds));
    }

    private PhotonTrackedTarget getTarget() {
        /* Make sure the target cache is incremented each loop */
        var previousTarget = targetCache.getAndIncrement();

        var results = new ArrayList<>(localization.getObjectTrackingResults(kObjectTrackingCameraName));

        /* Remove all results without targets - exit if no results have targets */
        results.removeIf((result) -> !result.hasTargets());
        if(results.isEmpty())
            return null;

        /* Find the most recent result */
        PhotonPipelineResult latestResult = results.get(0);
        for(var result : results) {
            if(result.getTimestampSeconds() > latestResult.getTimestampSeconds())
                latestResult = result;
        }

        /* Remove targets that do not match the id we are tracking - exit if the result has no targets we want */
        var filteredTargets = latestResult.getTargets();
        filteredTargets.removeIf((target) -> target.getDetectedObjectClassID() != objectToTrackId);

        if(filteredTargets.isEmpty())
            return null;

        PhotonTrackedTarget targetToTrack = null;
        /* Find the target that is closest to the center of the camera when there is no previous target */
        if(!targetCache.hasTarget()) {
            targetToTrack = latestResult.getBestTarget();
        }
        /* Find the lowest pitch difference target that is within the pitch difference cutoff */
        else {            
            double lowestPitchDifference = Double.MAX_VALUE;

            for(PhotonTrackedTarget target : filteredTargets) {
                double pitchDifference = Math.abs(target.getPitch() - previousTarget.getPitch());

                if(pitchDifference < kPitchDifferenceCutoff && pitchDifference < lowestPitchDifference) {
                    lowestPitchDifference = target.getPitch();

                    targetToTrack = target;
                }
            }
        }

        if (targetToTrack != null)
            targetCache.updateTarget(targetToTrack);

        return targetToTrack;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        rotationController.reset();
        targetCache.reset();

        swerve.drivetrain.setControl(kClosedLoopDriveRequest.withSpeeds(new ChassisSpeeds()));

        ElasticUtil.sendInfo("Finished object aiming");
    }
}