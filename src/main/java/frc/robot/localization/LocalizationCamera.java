package frc.robot.localization;

import java.util.List;
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

    private final boolean isObjectTracking;

    public LocalizationCamera(CameraConstants constants, AprilTagFieldLayout layout) {
        camera = new PhotonCamera(constants.name());

        poseEstimator = new PhotonPoseEstimator(
            layout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,  
            constants.robotToCameraTransform()
        );

        this.isObjectTracking = constants.isObjectTracking();
    }

    public void addResultsToDrivetrain(SwerveDrivetrain<?, ?, ?> drivetrain, Field2d localizationField) {
        /* This method should not run during object tracking */
        if (isObjectTracking) 
            return;

        var results = camera.getAllUnreadResults();

        for (PhotonPipelineResult pipelineResult : results) {
            Optional<EstimatedRobotPose> optionalResult = poseEstimator.update(pipelineResult);

            if (optionalResult.isPresent()) {
                EstimatedRobotPose poseEstimatorResult = optionalResult.get();

                double convertedTimestamp = Utils.fpgaToCurrentTime(poseEstimatorResult.timestampSeconds);

                Pose2d robotPose = poseEstimatorResult.estimatedPose.toPose2d();

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

    /** Returns all object tracking pipeline results. Do not call this during localization (or an empty list will be returned). */
    public List<PhotonPipelineResult> getObjectTrackingResults() {
        /* This method should not run during localization - return an empty list */
        if (!isObjectTracking)
            return List.of();

        return camera.getAllUnreadResults();
    }

    public String getName() {
        return camera.getName();
    }
}