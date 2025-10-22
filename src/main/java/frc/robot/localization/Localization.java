package frc.robot.localization;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.localization.TrackedObject.ObjectType;
import frc.robot.utils.ElasticUtil;

import static frc.robot.Constants.LocalizationConstants.*;
import java.util.List;
import java.util.Optional;

import org.ironmaple.simulation.SimulatedArena;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import java.util.ArrayList;

public class Localization {
    private final SwerveDrivetrain<?, ?, ?> drivetrain;

    private final Field2d field;

    private final List<LocalizationCamera> localizationCameras = new ArrayList<>();
    private final List<ObjectTrackingCamera> objectTrackingCameras = new ArrayList<>();

    private final List<TrackedObject> trackedObjects = new ArrayList<>();

    private final StructArrayPublisher<Pose3d> trackedObjectPublisher;

    private VisionSystemSim simulatedObjectTracking;
    private VisionSystemSim simulatedLocalization;

    private Pose2d predictedPose;
    private Pose2d currentPose = new Pose2d();
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

        if (RobotBase.isSimulation()) 
            initSimulation();

        else {
            for (var constants : kCameraConstants) {
                if (constants.isObjectTracking())
                    objectTrackingCameras.add(new ObjectTrackingCamera(constants));

                else
                    localizationCameras.add(new LocalizationCamera(constants, aprilTagLayout)); 
            }
        }

        SmartDashboard.putData("Localization Field", field);

        trackedObjectPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Tracked Objects", Pose3d.struct)
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
        var objectPoses = new Pose3d[trackedObjects.size()];
        for (int i = 0; i < trackedObjects.size(); i++) {
            objectPoses[i] = new Pose3d(trackedObjects.get(i).getFieldPosition());
        }

        trackedObjectPublisher.set(objectPoses);

        /* Update and publish the current pose estimate */
        var swerveStateCapture = drivetrain.getState();
        currentPose = swerveStateCapture.Pose;
        currentVelocity = swerveStateCapture.Speeds;

        field.setRobotPose(currentPose);

        SmartDashboard.putNumber("Current Speed", Math.hypot(currentVelocity.vxMetersPerSecond, currentVelocity.vyMetersPerSecond));

        /* Predict where the robot will be */
        predictedPose = currentPose.exp(currentVelocity.toTwist2d(kPoseLookaheadTime));
    } 

    private void initSimulation() {
        simulatedLocalization = new VisionSystemSim("Localization");
        simulatedLocalization.addAprilTags(aprilTagLayout);

        simulatedObjectTracking = new VisionSystemSim("Object Tracking");

        for (var constants : kCameraConstants) {
            if (constants.isObjectTracking()) {
                var camera = new ObjectTrackingCamera(constants);
                objectTrackingCameras.add(camera); 

                var properties = new SimCameraProperties();//TODO constants - there are more than this!
                properties.setCalibration(800, 600, Rotation2d.fromDegrees(70));
                properties.setCalibError(0.35, 0.10);
                properties.setFPS(30);
                properties.setAvgLatencyMs(20);
                properties.setLatencyStdDevMs(5);
                
                /* Links the simulated camera to the localization camera */
                var cameraSim = new PhotonCameraSim(camera.getPhotonCamera(), properties);
                cameraSim.enableDrawWireframe(false);
                cameraSim.enableRawStream(false);
                cameraSim.enableProcessedStream(true);

                simulatedObjectTracking.addCamera(cameraSim, constants.robotToCameraTransform());
            }

            else {
                var camera = new LocalizationCamera(constants, aprilTagLayout);
                localizationCameras.add(camera); 

                var properties = new SimCameraProperties();//TODO constants - there are more than this!
                properties.setCalibration(800, 600, Rotation2d.fromDegrees(60));
                properties.setCalibError(0.35, 0.10);
                properties.setFPS(30);
                properties.setAvgLatencyMs(20);
                properties.setLatencyStdDevMs(5);
                
                /* Links the simulated camera to the localization camera */
                var cameraSim = new PhotonCameraSim(camera.getPhotonCamera(), properties);
                cameraSim.enableDrawWireframe(false);
                cameraSim.enableRawStream(false);
                cameraSim.enableProcessedStream(false);

                simulatedLocalization.addCamera(cameraSim, constants.robotToCameraTransform());
            }
        }
    }

    /** Call this every loop to update simulation */
    public void updateSimulation(Pose2d simulatedRobotPose) {
        simulatedObjectTracking.clearVisionTargets();

        SimulatedArena.getInstance().getGamePiecesByType("Coral")//TODO don't clear objects every loop and add algae
            .stream()
            .map((coralPose) -> new VisionTargetSim(coralPose, kCoralModel))
            .forEach((target) -> simulatedObjectTracking.addVisionTargets("Coral", target));
        
        simulatedLocalization.update(simulatedRobotPose);
        simulatedObjectTracking.update(simulatedRobotPose);
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

    /** Gets the object with the specified tracking id, or returns an empty optional. */
    public Optional<TrackedObject> getTrackedObjectWithTrackingID(int id) {
        for (var object : trackedObjects) {
            if (object.getTrackingId() == id)
                return Optional.of(object);
        }

        return Optional.empty();
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

    public Pose2d getTagPose(int tagID) {
        return aprilTagLayout.getTagPose(tagID).get().toPose2d();
    }
}
   