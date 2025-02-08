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

        SmartDashboard.putBoolean(camera.getName() + " Present", false);
        SmartDashboard.putBoolean(camera.getName() + " Useable", false);

        for (PhotonPipelineResult pipelineResult : results) {
            Optional<EstimatedRobotPose> optionalResult = poseEstimator.update(pipelineResult);

            boolean isPresent = optionalResult.isPresent();

            SmartDashboard.putBoolean(camera.getName() + " Present", isPresent);

            if (isPresent) {
                EstimatedRobotPose poseEstimatorResult = optionalResult.get();
                
                double distance = getSmallestTagDistance(poseEstimatorResult);

                Pose2d robotPose = poseEstimatorResult.estimatedPose.toPose2d();

                /* Add this camera's pose to the field on the dashboard even if it isn't useable */
                localizationField.getObject(camera.getName()).setPose(robotPose);

                boolean isUseable = isUseableEstimate(poseEstimatorResult, distance);

                SmartDashboard.putBoolean(camera.getName() + " Usable", isUseable);

                if (isUseable) {
                    double convertedTimestamp = Utils.fpgaToCurrentTime(poseEstimatorResult.timestampSeconds);
 
                    var standardDeviations = calculateStandardDeviations(poseEstimatorResult, distance);

                    drivetrain.addVisionMeasurement(
                        robotPose,
                        convertedTimestamp,
                        standardDeviations
                    );

                    /* Add the standard deviations to the dashboard */
                    SmartDashboard.putNumberArray(camera.getName() + " StdDevs", standardDeviations.getData());
                }
            }

            else {
                SmartDashboard.putBoolean(camera.getName() + " Usable", false);

                /* Move pose off the field when an estimate is not present to avoid clutter */
                localizationField.getObject(camera.getName()).setPose(new Pose2d());
            }
        }
    }

    private boolean isUseableEstimate(EstimatedRobotPose estimation, double distance) {
        if (estimation.targetsUsed.size() > 1)
            return distance < kDistanceThreshold;

        double ambiguity = estimation.targetsUsed.get(0).poseAmbiguity;

        /* Display ambiguity on the dashboarad */
        SmartDashboard.putNumber(camera.getName() + " Ambiguity", ambiguity);

        return ambiguity < kAmbiguityThreshold && distance < kDistanceThreshold;
    }

    /** Finds the smallest distance to a tag */
    private double getSmallestTagDistance(EstimatedRobotPose estimation) {
        double smallestDistance = Double.POSITIVE_INFINITY;
        for (var target : estimation.targetsUsed) {
            var t3d = target.getBestCameraToTarget();

            var distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
            if (distance < smallestDistance)
                smallestDistance = distance;
        }

        /* Display distance on the dashboard */
        SmartDashboard.putNumber(camera.getName() + " Distance", smallestDistance);

        return smallestDistance;
    }

    /** Finds the standard deviations based on smallest tag distance, number of tags, and pose ambiguity  */
    private Matrix<N3, N1> calculateStandardDeviations(EstimatedRobotPose estimation, double smallestDistance) {
        /* Scale the deviations by distance if the smaller distance is beyond the noisy distance */
        double distanceMultiplier = 1;
        if (smallestDistance > kNoisyDistanceMeters) {
            distanceMultiplier = Math.max(1, 
                (smallestDistance - kNoisyDistanceMeters) * kDistanceWeight);
        }

        /* If there is only 1 tag there could be pose ambiguity, so increase standard deviations based on it */
        if (estimation.targetsUsed.size() == 1) {
            double ambiguity = estimation.targetsUsed.get(0).getPoseAmbiguity();

            double poseAmbiguityMultiplier = Math.max(1, 
                ambiguity * kPoseAmbiguityMultiplier);

            /* Combine distance and ambiguity deviation scaling */
            double confidenceMultiplier = distanceMultiplier * poseAmbiguityMultiplier;

            return kSingleTagBaseStandardDeviations.times(confidenceMultiplier);
        }

        else {
            /* If there are multiple tags, only scale based on distance */
            return kMultiTagBaseStandardDeviations.times(distanceMultiplier);
        }
    }
}