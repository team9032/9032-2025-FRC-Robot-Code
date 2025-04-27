package frc.robot.localization;

import static frc.robot.Constants.LocalizationConstants.kSameObjectDistance;

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
        Transform3d robotToCamera = constants.robotToCameraTransform();

        double targetYaw = Units.degreesToRadians(target.getYaw());

        /* Find the distance from the camera's lense to the object using target pitch and yaw */
        double cameraToTargetDistance =
            (-robotToCamera.getZ())
                / Math.tan(-robotToCamera.getRotation().getY() - Units.degreesToRadians(target.getPitch()))
                / Math.cos(-targetYaw);
        
        /* Find the camera's pose in field space using camera offsets and the robot's pose */
        Pose2d cameraInFieldSpace = poseAtDetectionTime.transformBy(
            new Transform2d(robotToCamera.getX(), robotToCamera.getY(), robotToCamera.getRotation().toRotation2d())   
        );

        /* Find the target's pose in field space by projecting outwards from the camera's pose in field space */
        Pose2d targetPoseInField =
            cameraInFieldSpace
                /* Rotate by the target yaw */
                .transformBy(new Transform2d(Translation2d.kZero, new Rotation2d(-targetYaw)))
                /* Project outwards using the camera to target distance */
                .transformBy(new Transform2d(new Translation2d(cameraToTargetDistance, 0), Rotation2d.kZero));

        boolean updatedObject = false;
        for (var object : objectList) {
            boolean withinSameDistance = object.getFieldPosition().getTranslation().getDistance(targetPoseInField.getTranslation()) < kSameObjectDistance;
            boolean sameType = object.getObjectType().equals(ObjectType.fromClassId(target.getDetectedObjectClassID()));

            /* If an object is close to a previous detection with the same object type, assume it's the same object 
                and update the previous detection */
            if (withinSameDistance && sameType) {
                object.update(targetPoseInField, target, getName(), timestamp);
 
                updatedObject = true;

                break;
            }
        }

        /* If a previous detection was not updated, create a new detection with a unique id */
        if (!updatedObject) {
            targetIdCounter++;

            objectList.add(new TrackedObject(targetPoseInField, target, getName(), timestamp, targetIdCounter));
        }
    }

    public String getName() {
        return camera.getName();
    }
}