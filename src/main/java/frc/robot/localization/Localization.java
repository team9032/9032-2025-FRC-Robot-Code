package frc.robot.localization;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.LocalizationConstants.*;

public class Localization {
    private final SwerveDrivetrain drivetrain;

    private final Field2d field;

    public Localization(SwerveDrivetrain drivetrain) {  
        this.drivetrain = drivetrain;

        field = new Field2d();

        SmartDashboard.putData("Localization Field", field);
    }

    /** Gets the unread results for each camera, then adds each result to its own photonPoseEstimator, 
     *  then adds the result of the photonPoseEstimator to the swerve one. Call this method once every loop. */
    public void update() {
        for (LocalizationCamera camera : kCameras) {
            camera.addResultsToDrivetrain(drivetrain, field);
        }

        field.setRobotPose(drivetrain.getState().Pose);
    }
}
   