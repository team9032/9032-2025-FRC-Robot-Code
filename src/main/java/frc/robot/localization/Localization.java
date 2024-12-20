package frc.robot.localization;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import static frc.robot.Constants.LocalizationConstants.*;

public class Localization {
    private final SwerveDrivetrain drivetrain;

    public Localization(SwerveDrivetrain drivetrain) {  
        this.drivetrain = drivetrain;
    }

    /** Gets the unread results for each camera, then adds each result to its own photonPoseEstimator, 
     *  then adds the result of the photonPoseEstimator to the swerve one. Call this method once every loop. */
    public void update() {
        for (LocalizationCamera camera : kCameras) {
            camera.addResultsToDrivetrain(drivetrain);
        }
    }
}
   