package frc.robot.localization;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.LocalizationConstants.*;

import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;

public class Localization {
    private final SwerveDrivetrain<?, ?, ?> drivetrain;

    private final Field2d field;

    private final LocalizationCamera[] cameras = new LocalizationCamera[kCameraConstants.length];

    public Localization(SwerveDrivetrain<?, ?, ?> drivetrain) {  
        this.drivetrain = drivetrain;

        field = new Field2d();

        AprilTagFieldLayout layout = null;
        try {
            layout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/" + kAprilTagFieldLayoutName);
        } catch(Exception e) {
            DriverStation.reportError("Error opening AprilTag field layout", e.getStackTrace());
        }

        for (int i = 0; i < cameras.length; i++) {
            cameras[i] = new LocalizationCamera(kCameraConstants[i], layout); 
        }

        SmartDashboard.putData("Localization Field", field);
    }

    /** Gets the unread results for each camera, then adds each result to its own photonPoseEstimator, 
     *  then adds the result of the photonPoseEstimator to the swerve one. Call this method once every loop. */
    public void updateLocalization() {
        for (LocalizationCamera camera : cameras) {
            camera.addResultsToDrivetrain(drivetrain, field);
        }

        field.setRobotPose(drivetrain.getState().Pose);
    }

    public void switchCameraToObjectTracking(String cameraName) {
        for(LocalizationCamera camera : cameras) {
            if(camera.getName().equals(cameraName)) 
                camera.switchToObjectTracking();
        }
    }
    
    /** Gets object tracking results from a camera. If the camera is not in object tracking mode, this will return null. */
    public List<PhotonPipelineResult> getObjectTrackingResults(String cameraName) {
        for(LocalizationCamera camera : cameras) {
            if(camera.getName().equals(cameraName)) 
                return camera.getObjectTrackingResults();
        }

        return null;
    }

    public void switchAllToLocalization() {
        for(LocalizationCamera camera : cameras)
            camera.switchToLocalization();
    }
}
   