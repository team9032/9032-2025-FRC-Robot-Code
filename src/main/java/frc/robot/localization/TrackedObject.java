package frc.robot.localization;

import static frc.robot.Constants.ObjectAimingConstants.kAlgaeId;
import static frc.robot.Constants.ObjectAimingConstants.kCoralId;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;

public class TrackedObject {
    private Pose2d fieldPosition;
    private PhotonTrackedTarget photonVisionData;
    private String cameraName;
    private double timestamp;

    private final int trackingId;

    public TrackedObject(Pose2d fieldPosition, PhotonTrackedTarget photonVisionData, String cameraName, double timestamp, int trackingId) {
        this.fieldPosition = fieldPosition;
        this.photonVisionData = photonVisionData;
        this.cameraName = cameraName;
        this.timestamp = timestamp;

        this.trackingId = trackingId;
    }

    public boolean isAlgae() {
        return photonVisionData.getDetectedObjectClassID() == kAlgaeId;
    }

    public boolean isCoral() {
        return photonVisionData.getDetectedObjectClassID()  == kCoralId;
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
