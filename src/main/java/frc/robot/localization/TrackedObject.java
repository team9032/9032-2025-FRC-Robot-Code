package frc.robot.localization;

import static frc.robot.localization.LocalizationConstants.kAlgaeHeight;
import static frc.robot.localization.LocalizationConstants.kAlgaeId;
import static frc.robot.localization.LocalizationConstants.kCoralHeight;
import static frc.robot.localization.LocalizationConstants.kCoralId;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;

public class TrackedObject {
    private Pose2d fieldPosition;
    private PhotonTrackedTarget photonVisionData;
    private String cameraName;
    private double timestamp;
    private ObjectType objectType;

    private final int trackingId;

    public static enum ObjectType {
        CORAL(kCoralHeight),
        ALGAE(kAlgaeHeight),
        UNKNOWN(0);

        private final double height;

        private ObjectType(double height) {
            this.height = height;
        }

        public static ObjectType fromClassId(int classId) {
            if (classId == kCoralId)
                return ObjectType.CORAL;
        
            else if (classId == kAlgaeId)
                return ObjectType.ALGAE;

            else 
                return ObjectType.UNKNOWN;
        }

        public double getHeight() {
            return height;
        }
    }

    public TrackedObject(Pose2d fieldPosition, PhotonTrackedTarget photonVisionData, String cameraName, double timestamp, int trackingId) {
        this.fieldPosition = fieldPosition;
        this.photonVisionData = photonVisionData;
        this.cameraName = cameraName;
        this.timestamp = timestamp;

        this.trackingId = trackingId;

        objectType = ObjectType.fromClassId(photonVisionData.getDetectedObjectClassID());//TODO fix class ids in sim
    }

    public boolean isCoral() {
        return objectType.equals(ObjectType.CORAL);
    }

    public boolean isAlgae() {
        return objectType.equals(ObjectType.ALGAE);
    }

    public ObjectType getObjectType() {
        return objectType;
    }

    public void update(Pose2d fieldPosition, PhotonTrackedTarget photonVisionData, String cameraName, double timestamp) {
        this.fieldPosition = fieldPosition;
        this.photonVisionData = photonVisionData;
        this.cameraName = cameraName;
        this.timestamp = timestamp;
    }

    public Pose2d getFieldPosition() {
        return fieldPosition;
    }

    public PhotonTrackedTarget getPhotonVisionData() {
        return photonVisionData;
    }

    public String getCameraName() {
        return cameraName;
    }

    public double getTimestamp() {
        return timestamp;
    }

    public int getTrackingId() {
        return trackingId;
    }
} 
