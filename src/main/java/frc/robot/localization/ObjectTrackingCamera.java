package frc.robot.localization;

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

public class ObjectTrackingCamera {
    private static int targetIdCounter = 0;

    private final PhotonCamera camera;

    private final CameraConstants constants;

    public ObjectTrackingCamera(CameraConstants constants) {
        camera = new PhotonCamera(constants.name());

        this.constants = constants;
    }

    /** Adds new detected objected to the list and updates previous detections if they have changed */
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

        double cameraToTargetDistance =
            (-robotToCamera.getZ())
                / Math.tan(-robotToCamera.getRotation().getY() - Units.degreesToRadians(target.getPitch()))
                / Math.cos(-targetYaw);
        
        Pose2d fieldToCamera = poseAtDetectionTime.transformBy(
            new Transform2d(robotToCamera.getX(), robotToCamera.getY(), robotToCamera.getRotation().toRotation2d())   
        );

        Pose2d targetPoseInField =
            fieldToCamera
                .transformBy(new Transform2d(Translation2d.kZero, new Rotation2d(-targetYaw)))
                .transformBy(new Transform2d(new Translation2d(cameraToTargetDistance, 0), Rotation2d.kZero));

        boolean updatedObject = false;
        for (var object : objectList) {
            if (object.getFieldPosition().getTranslation().getDistance(targetPoseInField.getTranslation()) < Units.inchesToMeters(6)) {//TODO make constant
                object.update(targetPoseInField, target, getName(), timestamp);

                updatedObject = true;

                break;
            }
        }

        if (!updatedObject) {
            targetIdCounter++;

            objectList.add(new TrackedObject(targetPoseInField, target, getName(), timestamp, targetIdCounter));
        }
    }

    public String getName() {
        return camera.getName();
    }
}