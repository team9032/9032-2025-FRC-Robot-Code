package frc.robot.localization;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.localization.TrackedObject.ObjectType;
import frc.robot.utils.ElasticUtil;

import static frc.robot.Constants.LocalizationConstants.*;
import java.util.List;
import java.util.Optional;
import java.util.ArrayList;

public class Localization {
    private final SwerveDrivetrain<?, ?, ?> drivetrain;

    private final Field2d field;

    private final List<LocalizationCamera> localizationCameras = new ArrayList<>();
    private final List<ObjectTrackingCamera> objectTrackingCameras = new ArrayList<>();

    private final List<TrackedObject> trackedObjects = new ArrayList<>();

    private final StructArrayPublisher<Pose2d> trackedObjectPublisher;

    private Pose2d predictedPose;
    private Pose2d currentPose;
    private ChassisSpeeds currentVelocity;

    private AprilTagFieldLayout aprilTagLayout;

    public Localization(SwerveDrivetrain<?, ?, ?> drivetrain) {  
        this.drivetrain = drivetrain;

        field = new Field2d();

        try {
            aprilTagLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/" + kAprilTagFieldLayoutName);
        } catch(Exception e) {
            aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

            ElasticUtil.sendError("Error opening AprilTag field layout", "Localization will use the default layout");
        }

        for (var constants : kCameraConstants) {
            if (constants.isObjectTracking())
                objectTrackingCameras.add(new ObjectTrackingCamera(constants));

            else
                localizationCameras.add(new LocalizationCamera(constants, aprilTagLayout)); 
        }

        SmartDashboard.putData("Localization Field", field);

        trackedObjectPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Tracked Objects", Pose2d.struct)
            .publish();
    }

    /** For pose estimation, gets the unread results for each camera, then adds each result to its own photonPoseEstimator, 
     *  then adds the result of the photonPoseEstimator to the swerve one. 
     *  For object tracking, gets the latest object tracking results from each camera, estimates the objects position on the field, and updates the list of tracked objects.
     *  Call this method once every loop. */
    public void updateLocalization() {
        /* Update all pose estimation cameras */
        for (LocalizationCamera camera : localizationCameras) {
            camera.addResultsToDrivetrain(drivetrain, field);
        }

        /* Update all object tracking cameras */
        for (ObjectTrackingCamera camera : objectTrackingCameras) {
            camera.addResultsToObjectList(drivetrain, trackedObjects);
        }

        /* Remove any old objects */
        double currentTime = Utils.getCurrentTimeSeconds();
        trackedObjects.removeIf((object) -> currentTime - object.getTimestamp() > kObjectExpireTime);

        /* Remove objects that are off the field */
        trackedObjects.removeIf((object) -> !isPoseOnField(object.getFieldPosition()));

        /* Publish each object's pose */
        trackedObjectPublisher.set((Pose2d[]) 
            trackedObjects
                .stream()
                .map((object) -> object.getFieldPosition())
                .toArray()
        );

        /* Update and publish the current pose estimate */
        var swerveStateCapture = drivetrain.getState();
        currentPose = swerveStateCapture.Pose;
        currentVelocity = swerveStateCapture.Speeds;

        field.setRobotPose(currentPose);

        if (!trackedObjects.isEmpty()) {//TODO find a better way to visualize and remove this
            var object = trackedObjects.get(0);

            field.getObject(object.getCameraName() + " target").setPose(object.getFieldPosition());

            SmartDashboard.putString("object 0", object.getTrackingId() + " " + object.getCameraName() + " " + object.getFieldPosition());
        }

        /* Predict where the robot will be */
        predictedPose = currentPose.exp(currentVelocity.toTwist2d(kPoseLookaheadTime));
    } 

    /** Gets the current pose */
    public Pose2d getCurrentPose() {
        return currentPose;
    }

    /** Gets the pose that the robot will likely be at in the near future based on the current velocity */
    public Pose2d getPredictedPose() {
        return predictedPose;
    }

    /** Gets the current velocity */
    public ChassisSpeeds getCurrentVelocity() {
        return currentVelocity;
    }

    /** Finds the nearest object of the given type. If no objects of that type are detected, this returns an empty optional. */
    public Optional<TrackedObject> getNearestObjectOfType(ObjectType objectType) {
        var trackedObjects = getTrackedObjectsOfType(objectType);
        if (trackedObjects.isEmpty())
            return Optional.empty();

        var currentTranslation = drivetrain.getState().Pose.getTranslation(); 

        double closestDistance = Double.MAX_VALUE;
        TrackedObject closestObject = trackedObjects.get(0);
        for (var object : trackedObjects) {
            double distance = object.getFieldPosition().getTranslation().getDistance(currentTranslation);

            if (distance < closestDistance) {
                closestObject = object;

                closestDistance = distance;
            }
        }

        return Optional.of(closestObject);
    } 

    public List<TrackedObject> getTrackedObjectsOfType(ObjectType objectType) {
        return trackedObjects
            .stream()
            .filter((object) -> object.getObjectType().equals(objectType))
            .toList();
    }

    public List<TrackedObject> getTrackedObjectsFromCamera(String cameraName) {
        return trackedObjects
            .stream()
            .filter((object) -> object.getCameraName().equals(cameraName))
            .toList();
    }

    public List<TrackedObject> getTrackedObjectsFromCameraWithType(String cameraName, ObjectType objectType) {
        return trackedObjects
            .stream()
            .filter((object) -> object.getCameraName().equals(cameraName) && object.getObjectType().equals(objectType))
            .toList();
    }

    public boolean isPoseOnField(Pose2d pose) {
        /* Poses use a global blue origin with (0, 0) at the lower left */
        return 0 < pose.getY() && pose.getY() < aprilTagLayout.getFieldWidth()
            && 0 < pose.getX() && pose.getX() < aprilTagLayout.getFieldLength();
    }
}
   