package frc.robot.localization;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.LocalizationConstants.*;

public class Localization {
    private final SwerveDrivetrain<?, ?, ?> drivetrain;

    private final Field2d field;

    private final LocalizationCamera[] cameras = new LocalizationCamera[kCameraConstants.length];

    public Localization(SwerveDrivetrain<?, ?, ?> drivetrain) {  
        this.drivetrain = drivetrain;

        field = new Field2d();

        try {
            AprilTagFieldLayout layout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/" + kAprilTagFieldLayoutName);

            for (int i = 0; i < cameras.length; i++) {
                cameras[i] = new LocalizationCamera(kCameraConstants[i], layout); 
            }
        } catch(Exception e) {
            DriverStation.reportError("Error opening AprilTag field layout - Localization will commit die!", e.getStackTrace());
        }

        SmartDashboard.putData("Localization Field", field);
    }

    /** Gets the unread results for each camera, then adds each result to its own photonPoseEstimator, 
     *  then adds the result of the photonPoseEstimator to the swerve one. Call this method once every loop. */
    public void update() {
        for (LocalizationCamera camera : cameras) {
            camera.addResultsToDrivetrain(drivetrain, field);
        }

        field.setRobotPose(drivetrain.getState().Pose);
    }
}
   