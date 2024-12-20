package frc.robot.localization;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import static frc.robot.Constants.LocalizationConstants.*;

public class LocalizationCamera {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    private final Field2d field;

    public LocalizationCamera(String name, Transform3d robotToCameraTransform) {
        camera = new PhotonCamera(name);
        poseEstimator = new PhotonPoseEstimator(
            kAprilTagFieldLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,  
            robotToCameraTransform
        );   

        field = new Field2d();
    }

    public void addResultsToDrivetrain(SwerveDrivetrain drivetrain) {
        var results = camera.getAllUnreadResults();

        for (PhotonPipelineResult result : results) {
            Optional<EstimatedRobotPose> optionalEstimatedPose = poseEstimator.update(result);

            if (optionalEstimatedPose.isPresent()) {
                EstimatedRobotPose estimatedPose = optionalEstimatedPose.get();

                drivetrain.addVisionMeasurement(
                    estimatedPose.estimatedPose.toPose2d(),
                    estimatedPose.timestampSeconds,
                    confidenceCalculator(estimatedPose)
                );

                field.setRobotPose(estimatedPose.estimatedPose.toPose2d());
            }
        }
    }

    /** Finds the standard deviations based on smallest tag distance, number of tags, and pose ambiguity  */
    private Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation) {//TODO test this method
        /* Find the smallest distance to a tag */
        double smallestDistance = Double.POSITIVE_INFINITY;
        for (var target : estimation.targetsUsed) {
            var t3d = target.getBestCameraToTarget();

            var distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
            if (distance < smallestDistance)
                smallestDistance = distance;
        }

        /* If there is only 1 tag, the pose ambiguity is high, so increase standard deviations */
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

        return kVisionStandardDeviations.times(confidenceMultiplier);
    }
}