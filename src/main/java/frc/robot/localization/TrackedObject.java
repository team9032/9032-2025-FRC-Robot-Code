package frc.robot.localization;

import static frc.robot.Constants.ObjectAimingConstants.kAlgaeId;
import static frc.robot.Constants.ObjectAimingConstants.kCoralId;

import edu.wpi.first.math.geometry.Pose2d;

public record TrackedObject(Pose2d position, double objectYawInFieldSpace, double objectPitchInCameraSpace, int id, String cameraName) {
    public boolean isAlgae() {
        return id == kAlgaeId;
    }
    public boolean isCoral() {
        return id == kCoralId;
    }
} 
