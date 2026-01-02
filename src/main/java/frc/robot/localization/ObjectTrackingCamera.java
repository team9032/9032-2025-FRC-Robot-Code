package frc.robot.localization;

import static frc.robot.localization.LocalizationConstants.kSameObjectDistance;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.localization.TrackedObject.ObjectType;

public class ObjectTrackingCamera {
    private static int targetIdCounter = 0;

    private final PhotonCamera camera;

    private final CameraConstants constants;

    public ObjectTrackingCamera(CameraConstants constants) {
        camera = new PhotonCamera(constants.name());

        this.constants = constants;
    }

    /** Adds new detected objects to the list and updates previous detections if they have changed */
    public void addResultsToObjectList(SwerveDrivetrain<?, ?, ?> drivetrain, List<TrackedObject> objectList) {
        var results = camera.getAllUnreadResults();

        for (var result : results) {
            double timestamp = Utils.fpgaToCurrentTime(result.getTimestampSeconds());

            var optionalPose = drivetrain.samplePoseAt(timestamp);

            if (optionalPose.isPresent()) {
                for (var target : result.getTargets())
                    processTarget(target, optionalPose.get(), objectList, timestamp);
            }
        }
    }

    private void processTarget(PhotonTrackedTarget target, Pose2d poseAtDetectionTime, List<TrackedObject> objectList, double timestamp) {
        var targetType = ObjectType.fromClassId(target.getDetectedObjectClassID());
        var targetPose = getTargetPoseInFieldSpace(target, poseAtDetectionTime, targetType.getHeight());

        boolean updatedObject = false;
        for (var object : objectList) {
            boolean withinSameDistance = object.getFieldPosition().getTranslation().getDistance(targetPose.getTranslation()) < kSameObjectDistance;
            boolean sameType = object.getObjectType().equals(targetType);//TODO fix class ids in sim

            /* If an object is close to a previous detection with the same object type, assume it's the same object 
                and update the previous detection */
            if (withinSameDistance && sameType) {
                object.update(targetPose, target, getName(), timestamp);
 
                updatedObject = true;

                break;
            }
        }

        /* If a previous detection was not updated, create a new detection with a unique id */
        if (!updatedObject) {
            targetIdCounter++;

            objectList.add(new TrackedObject(targetPose, target, getName(), timestamp, targetIdCounter));
        }
    }

    private Pose2d getTargetPoseInFieldSpace(PhotonTrackedTarget target, Pose2d robotPose, double targetHeight) {
        Transform3d robotToCamera = constants.robotToCameraTransform();
        
        /* Find target pitch and yaw in robot space by applying the camera's rotation offsets */
        double targetYawInRobotSpace = -robotToCamera.getRotation().getZ() - Units.degreesToRadians(target.getYaw());
        double targetPitchInRobotSpace = robotToCamera.getRotation().getY() - Units.degreesToRadians(target.getPitch());

        double cameraHeightAboveTarget = robotToCamera.getZ() - (targetHeight / 2.0);//TODO compensate for lollipop coral height

        /* Find the magnitude of the 2d vector that points from the 2d location of the camera to the target */
        double distanceToTargetOnFieldPlane = cameraHeightAboveTarget / Math.tan(targetPitchInRobotSpace);

        /* Find the x and y components of the vector and apply the camera's translation offsets to find the target in robot space */
        double x = (distanceToTargetOnFieldPlane * Math.cos(targetYawInRobotSpace)) + robotToCamera.getX();
        double y = (distanceToTargetOnFieldPlane * Math.sin(targetYawInRobotSpace)) + robotToCamera.getY();
        var targetInRobotSpace = new Translation2d(x, y);

        /* Transform the robot's pose by the target's location in robot space to find the target's location in field space */
        Translation2d targetInFieldSpace = robotPose.transformBy(new Transform2d(targetInRobotSpace, Rotation2d.kZero)).getTranslation();

        return new Pose2d(targetInFieldSpace, Rotation2d.kZero);//TODO rotate using bounding box
    }

    public String getName() {
        return camera.getName();
    }

    public PhotonCamera getPhotonCamera() {
        return camera;
    }
}