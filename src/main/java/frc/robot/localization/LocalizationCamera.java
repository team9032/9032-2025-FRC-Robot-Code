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
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.localization.LocalizationConstants.*;

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
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public void addResultsToDrivetrain(SwerveDrivetrain<?, ?, ?> drivetrain, Field2d localizationField) {
        /* Add heading data using the same timestamp units as the pipeline result - this must not use current time! */
        poseEstimator.addHeadingData(Timer.getFPGATimestamp(), drivetrain.getState().Pose.getRotation());

        var results = camera.getAllUnreadResults();

        for (PhotonPipelineResult pipelineResult : results) {
            boolean isMultitag = pipelineResult.targets.size() > 1;

            /* There is no ambiguity for multitag or empty results */
            double ambiguity = isMultitag || pipelineResult.targets.isEmpty() ? 0.0 : pipelineResult.targets.get(0).getPoseAmbiguity();

            /* If ambiguity is too high, use trig estimation since it is not affected by ambiguity */
            boolean useTrig = ambiguity > kAmbiguityThresholdForTrig;
            poseEstimator.setPrimaryStrategy(useTrig ? PoseStrategy.PNP_DISTANCE_TRIG_SOLVE : PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
                   
            Optional<EstimatedRobotPose> optionalResult = poseEstimator.update(pipelineResult);
            if (optionalResult.isPresent()) {
                EstimatedRobotPose poseEstimatorResult = optionalResult.get();
                
                Pose2d robotPose = poseEstimatorResult.estimatedPose.toPose2d();

                /* Add this camera's pose to the field on the dashboard even if it isn't useable */
                localizationField.getObject(camera.getName()).setPose(robotPose);

                /* Discard results that are too far away */
                double distance = getSmallestTagDistance(poseEstimatorResult);
                boolean isUseable = distance < kDistanceThreshold;

                if (isUseable) {
                    double convertedTimestamp = Utils.fpgaToCurrentTime(poseEstimatorResult.timestampSeconds);
 
                    var standardDeviations = calculateStandardDeviations(poseEstimatorResult, distance, useTrig);

                    drivetrain.addVisionMeasurement(
                        robotPose,
                        convertedTimestamp,
                        standardDeviations
                    );

                    /* Add the standard deviations to the dashboard */
                    SmartDashboard.putNumberArray(camera.getName() + " StdDevs", standardDeviations.getData());
                }

                SmartDashboard.putBoolean(camera.getName() + " Usable", isUseable);
                SmartDashboard.putNumber(camera.getName() + " Distance", distance);
            }

            else {
                SmartDashboard.putBoolean(camera.getName() + " Usable", false);
                SmartDashboard.putNumber(camera.getName() + " Distance", 0.0);

                /* Move pose off the field when an estimate is not present to avoid clutter */
                localizationField.getObject(camera.getName()).setPose(new Pose2d());
            }

            SmartDashboard.putNumber(camera.getName() + " Ambiguity", ambiguity);
            SmartDashboard.putBoolean(camera.getName() + " Present", optionalResult.isPresent());
        }
    }

    /** Finds the smallest distance to a tag */
    private double getSmallestTagDistance(EstimatedRobotPose estimation) {
        double smallestDistance = Double.POSITIVE_INFINITY;
        for (var target : estimation.targetsUsed) {
            double distance = target.getBestCameraToTarget().getTranslation().getNorm();

            if (distance < smallestDistance)
                smallestDistance = distance;
        }

        return smallestDistance;
    }

    /** Finds the standard deviations based on smallest tag distance, number of tags, if the pose was calculated using trig, and pose ambiguity  */
    private Matrix<N3, N1> calculateStandardDeviations(EstimatedRobotPose estimation, double smallestDistance, boolean useTrig) {
        /* Scale the deviations by distance if the smaller distance is beyond the noisy distance */
        double distanceMultiplier = 1.0;
        if (smallestDistance > kNoisyDistanceMeters) {
            distanceMultiplier = Math.max(1.0, (smallestDistance - kNoisyDistanceMeters) * kDistanceWeight);
        }

        /* Ambiguity does not affect trig estimation. Heading cannot be updated, so make heading standard deviation super high */
        if (useTrig) {
            return VecBuilder.fill(kTrigBaseXStandardDeviation, kTrigBaseYStandardDeviation, 999999999.0)
                .times(distanceMultiplier);
        }

        /* If there is only 1 tag there could be pose ambiguity, so increase standard deviations based on it */
        if (estimation.targetsUsed.size() == 1) {
            double ambiguity = estimation.targetsUsed.get(0).getPoseAmbiguity();

            double poseAmbiguityMultiplier = Math.max(1, ambiguity * kPoseAmbiguityMultiplier);

            /* Combine distance and ambiguity deviation scaling */
            double confidenceMultiplier = distanceMultiplier * poseAmbiguityMultiplier;

            return kSingleTagPNPBaseStandardDeviations.times(confidenceMultiplier);
        }

        else {
            /* If there are multiple tags, only scale based on distance */
            return kMultiTagBaseStandardDeviations.times(distanceMultiplier);
        }
    }

    public String getName() {
        return camera.getName();
    }

    public PhotonCamera getPhotonCamera() {
        return camera;
    }
}