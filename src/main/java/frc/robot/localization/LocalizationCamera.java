package frc.robot.localization;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.LocalizationConstants.*;

public class LocalizationCamera {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    private final CameraConstants constants;

    private final List<TrackedObject> trackedObjects = new ArrayList<>();

    public LocalizationCamera(CameraConstants constants, AprilTagFieldLayout layout) {
        camera = new PhotonCamera(constants.name());

        this.constants = constants;

        poseEstimator = new PhotonPoseEstimator(
            layout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,  
            constants.robotToCameraTransform()
        );
    }

    public void addResultsToDrivetrain(SwerveDrivetrain<?, ?, ?> drivetrain, Field2d localizationField) {
        /* This method should not run during object tracking */
        if (constants.isObjectTracking()) 
            return;

        var results = camera.getAllUnreadResults();

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

    /** Updates the object tracking pipeline. Will only have an effect if the camera is on object tracking mode. */
    public void updateObjectTrackingResults(Pose2d currentEstimatedPose, Field2d field) {
        /* This method should not run during localization */
        if (!constants.isObjectTracking())
            return;

        trackedObjects.clear();//TODO persist objects

        var result = getLatestPipelineResult();

        if (result == null)
            return;

        var cameraTransform = constants.robotToCameraTransform();

        for (var target : result.getTargets()) {
            /* Finds the distance from the camera to the object - no offsets are applied */
            double cameraToTargetDistance = 2;/*PhotonUtils.calculateDistanceToTargetMeters(
                cameraTransform.getZ(), 
                0.0, //TODO height
                -cameraTransform.getRotation().getY(), 
                target.getPitch()
            );*/
//TODO this code really doesn't work
            /* Estimate the yaw in field space for use with auto aiming */
            double objectYawInRobotSpace = target.getYaw(); //- Units.radiansToDegrees(cameraTransform.getRotation().getZ());//TODO add or subtract?
            double objectYawInFieldSpace = currentEstimatedPose.getRotation().getDegrees() - objectYawInRobotSpace;

            /* Finds the translation (x, y) from the camera to the target using distance - no offsets */
            var objectTranslationCamera = PhotonUtils.estimateCameraToTargetTranslation(cameraToTargetDistance, Rotation2d.fromDegrees(target.getYaw()));

            /* Finds the translation (x, y) in robot space using camera offsets */
            var objectTranslationRobot = objectTranslationCamera.minus(cameraTransform.getTranslation().toTranslation2d());

            /* Finds the translation (x, y) in field space by offseting by the robot's rotation and translation in the field */
            var objectTranslationField = objectTranslationRobot.rotateBy(currentEstimatedPose.getRotation())//(Rotation2d.fromDegrees(objectYawInFieldSpace))
                .plus(currentEstimatedPose.getTranslation());

            /* Create a pose estimate assuming the rotation is 0 */
            Pose2d objectPoseField = new Pose2d(objectTranslationField, Rotation2d.kZero);

            trackedObjects.add(new TrackedObject(objectPoseField, objectYawInFieldSpace, target.getPitch(), target.objDetectId, getName())); 
            // field.getObject(getName() + " Coral").setPose(objectPoseField);//TODO label coral
        }
    }

    private PhotonPipelineResult getLatestPipelineResult() {
        var results = new ArrayList<>(camera.getAllUnreadResults());

        /* Remove all results without targets */
        results.removeIf((result) -> !result.hasTargets());

        if (results.isEmpty())
            return null;

        /* Find the most recent result */
        PhotonPipelineResult latestResult = results.get(0);
        for(var result : results) {
            if(result.getTimestampSeconds() > latestResult.getTimestampSeconds())
                latestResult = result;
        }

        return latestResult;        
    }

    public List<TrackedObject> getLatestTrackedObjects() {
        return new ArrayList<>(trackedObjects);
    }

    public String getName() {
        return camera.getName();
    }
}