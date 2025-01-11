package frc.robot.localization;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.LocalizationConstants.*;

public class LocalizationCamera {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    public LocalizationCamera(CameraConstants constants, AprilTagFieldLayout layout) {
        camera = new PhotonCamera(constants.name());

        poseEstimator = new PhotonPoseEstimator(
            layout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,  
            constants.robotToCameraTransform()
        );
    }

    public void addResultsToDrivetrain(SwerveDrivetrain<?, ?, ?> drivetrain, Field2d localizationField) {
        var results = camera.getAllUnreadResults();

        for (PhotonPipelineResult pipelineResult : results) {
            Optional<EstimatedRobotPose> optionalResult = poseEstimator.update(pipelineResult);

            if (optionalResult.isPresent()) {
                EstimatedRobotPose poseEstimatorResult = optionalResult.get();

                double convertedTimestamp = Utils.fpgaToCurrentTime(poseEstimatorResult.timestampSeconds);

                Pose2d robotPose = removeAmbiguity(drivetrain, poseEstimatorResult, convertedTimestamp);

                var standardDeviations = confidenceCalculator(poseEstimatorResult);

                drivetrain.addVisionMeasurement(
                    robotPose,
                    convertedTimestamp,
                    standardDeviations
                );

                /* Add this camera's pose to the field on the dashboard */
                localizationField.getObject(camera.getName()).setPose(robotPose);

                /* Add the standard deviations to the dashboard */
                SmartDashboard.putNumberArray(camera.getName() + " StdDevs", standardDeviations.getData());
            }
        }
    }

    /** Attempts to remove pose ambiguity for single tag measurements by comparing the best and alternate pose heading 
     *  to the drivetrain's heading and using the one with the least difference - does nothing if there are multiple tags or low ambiguity */
    private Pose2d removeAmbiguity(SwerveDrivetrain<?, ?, ?> drivetrain, EstimatedRobotPose estimatorResult, double timestamp) {
        /* By default, the pose estimator uses the best pose */
        Pose2d bestReprojPose = estimatorResult.estimatedPose.toPose2d();;

        if(estimatorResult.targetsUsed.size() == 1) {
            var target = estimatorResult.targetsUsed.get(0);

            if(target.getPoseAmbiguity() > kAcceptablePoseAmbiguity) {
                Optional<Pose2d> optionalDrivetrainPose = drivetrain.samplePoseAt(timestamp);

                if(optionalDrivetrainPose.isPresent()) {
                    var drivetrainPose = optionalDrivetrainPose.get();
                    
                    var tagPose = poseEstimator.getFieldTags().getTagPose(target.fiducialId).get();

                    /* Convert the alternate transform to a pose based on what is done in photonPoseEstimator */
                    Pose2d alternateReprojPose = tagPose.transformBy(target.getAlternateCameraToTarget().inverse())
                        .transformBy(poseEstimator.getRobotToCameraTransform().inverse())
                        .toPose2d();
        
                    /* Find the differences between the drivetrain's pose and the estimates */
                    double bestAngleDifference = Math.abs(bestReprojPose.minus(drivetrainPose).getRotation().getDegrees());
                    double alternateAngleDifference = Math.abs(alternateReprojPose.minus(drivetrainPose).getRotation().getDegrees());
                    
                    /* Only return the alternate pose if it has a lower difference than the best pose */
                    if(bestAngleDifference > alternateAngleDifference) {
                        return alternateReprojPose;
                    }
                }
            }
        }

        return bestReprojPose;
    }

    /** Finds the standard deviations based on smallest tag distance, number of tags, and pose ambiguity  */
    private Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation) {
        /* Find the smallest distance to a tag */
        double smallestDistance = Double.POSITIVE_INFINITY;
        for (var target : estimation.targetsUsed) {
            var t3d = target.getBestCameraToTarget();

            var distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
            if (distance < smallestDistance)
                smallestDistance = distance;
        }

        /* If there is only 1 tag there could be pose ambiguity, so increase standard deviations based on it */
        double poseAmbiguityMultiplier = 1;
        if (estimation.targetsUsed.size() == 1) {
            poseAmbiguityMultiplier = Math.max(1, 
                (estimation.targetsUsed.get(0).getPoseAmbiguity() + kPoseAmbiguityOffset) * kPoseAmbiguityMultiplier);
        }

        /* Scale the deviations by distance if the smaller distance is beyond the noisy distance */
        double distanceMultiplier = 1;
        if (smallestDistance > kNoisyDistanceMeters) {
            distanceMultiplier = Math.max(1, 
                (smallestDistance - kNoisyDistanceMeters) * kDistanceWeight);
        }

        /* If there is more than 1 tag, lower the deviations based on number of tags */
        double tagPresenceDivider = 1 + ((estimation.targetsUsed.size() - 1) * kTagPresenceWeight);

        /* Combine all standard deviation scaling */
        double confidenceMultiplier = Math.max(1, (distanceMultiplier * poseAmbiguityMultiplier) / tagPresenceDivider);

        return kBaseStandardDeviations.times(confidenceMultiplier);
    }
}