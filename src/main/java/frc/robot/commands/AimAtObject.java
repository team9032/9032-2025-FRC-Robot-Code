package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.localization.Localization;
import frc.robot.subsystems.swerve.KrakenSwerve;
import frc.robot.utils.VisionTargetCache;

import static frc.robot.Constants.ObjectAimingConstants.*;
import static frc.robot.Constants.PathplannerConfig.kPathPlannerDriveRequest;

import java.util.ArrayList;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AimAtObject extends Command {
    private final KrakenSwerve swerve;
    
    private final Localization localization;

    private final VisionTargetCache<PhotonTrackedTarget> targetCache;
    
    private final PIDController rotationController;

    private final int objectToTrackId;    

    public AimAtObject(KrakenSwerve swerve, int objectToTrackId) {
       this.swerve = swerve; 
       this.objectToTrackId = objectToTrackId;

       localization = swerve.getLocalization();

       rotationController = new PIDController(kPRotation, 0.0, kDRotation);
       rotationController.setSetpoint(kRotationSetpoint);

       targetCache = new VisionTargetCache<>(kCycleAmtSinceTargetSeenCutoff);

       addRequirements(swerve);
    }

    @Override
    public void initialize() {}  

    @Override
    public void execute() {
        /* Make sure the target cache is incremented each loop */
        var previousTarget = targetCache.getAndIncrement();

        var results = new ArrayList<>(localization.getObjectTrackingResults(kObjectTrackingCameraName));

        /* Remove all results without targets - exit if no results have targets */
        results.removeIf((result) -> !result.hasTargets());
        if(results.isEmpty())
            return;

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
            return;

        PhotonTrackedTarget targetToTrack = null;
        /* Find the target that is closest to the center of the camera when there is no previous target */
        if(!targetCache.hasTarget()) 
            targetToTrack = latestResult.getBestTarget();
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

        if (targetToTrack == null)
            return;

        /* Drive based on target yaw */
        var speeds = new ChassisSpeeds(
            -kDrivingSpeed,
            0.0, 
            rotationController.calculate(targetToTrack.yaw)
        );

        swerve.drivetrain.setControl(kPathPlannerDriveRequest.withSpeeds(speeds));

        targetCache.updateTarget(targetToTrack);
    }

    @Override
    public boolean isFinished() {
        return targetCache.targetExpired();
    }

    @Override
    public void end(boolean interrupted) {
        rotationController.reset();
        targetCache.reset();

        swerve.drivetrain.setControl(kPathPlannerDriveRequest.withSpeeds(new ChassisSpeeds()));
    }
}