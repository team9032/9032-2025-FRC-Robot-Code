package frc.robot.localization;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.ElasticUtil;

import static frc.robot.Constants.LocalizationConstants.*;
import java.util.List;

import java.util.ArrayList;

public class Localization {
    private final SwerveDrivetrain<?, ?, ?> drivetrain;

    private final Field2d field;

    private final List<LocalizationCamera> localizationCameras = new ArrayList<>();
    private final List<ObjectTrackingCamera> objectTrackingCameras = new ArrayList<>();

    private final List<TrackedObject> trackedObjects = new ArrayList<>();

    private final StructArrayPublisher<Pose2d> trackedObjectPublisher;

    public Localization(SwerveDrivetrain<?, ?, ?> drivetrain) {  
        this.drivetrain = drivetrain;

        field = new Field2d();

        try {
            AprilTagFieldLayout layout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/" + kAprilTagFieldLayoutName);

            for (var constants : kCameraConstants) {
                if (constants.isObjectTracking())
                    objectTrackingCameras.add(new ObjectTrackingCamera(constants));

                else
                    localizationCameras.add(new LocalizationCamera(constants, layout)); 
            }
        } catch(Exception e) {
            ElasticUtil.sendError("Error opening AprilTag field layout", "Localization will commit die!");
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

        var currentPose = drivetrain.getState().Pose;
        field.setRobotPose(currentPose);

        /* Update all object tracking cameras */
        for (ObjectTrackingCamera camera : objectTrackingCameras) {
            camera.addResultsToObjectList(drivetrain, trackedObjects);
        }

        /* Remove any old objects */
        double currentTime = Utils.getCurrentTimeSeconds();
        trackedObjects.removeIf((object) -> currentTime - object.getTimestamp() > kObjectExpireTime);

        /* Publish each object's pose */
        trackedObjectPublisher.set((Pose2d[]) 
            trackedObjects
                .stream()
                .map((object) -> object.getFieldPosition())
                .toArray()
        );

        if (!trackedObjects.isEmpty()) {
            var object = trackedObjects.get(0);

            field.getObject(object.getCameraName() + " target").setPose(object.getFieldPosition());//TODO find a better way to visualize

            SmartDashboard.putString("object 0", object.getTrackingId() + " " + object.getCameraName() + " " + object.getFieldPosition());//TODO find a better way to show
        }
    } 

    public List<TrackedObject> getTrackedCoral() {
        return trackedObjects
            .stream()
            .filter((object) -> object.isCoral())
            .toList();
    }

    public List<TrackedObject> getTrackedAlgae() {
        return trackedObjects
            .stream()
            .filter((object) -> object.isAlgae())
            .toList();
    }

    public List<TrackedObject> getTrackedObjectsFromCamera(String cameraName) {
        return trackedObjects
            .stream()
            .filter((object) -> object.getCameraName().equals(cameraName))
            .toList();
    }
}
   