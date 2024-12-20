package frc.robot;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import static frc.robot.Constants.LocalizationConstants.*;

import java.util.Optional;

public class Localization {
    private final Field2d[] fields = new Field2d[kNumberCameras];

    private final PhotonCamera[] cameras = new PhotonCamera[kNumberCameras];
    private final PhotonPoseEstimator[] photonPoseEstimators = new PhotonPoseEstimator[kNumberCameras];

    private final SwerveDrivetrain drivetrain;

    public Localization(SwerveDrivetrain drivetrain) {  
        for(int i = 0; i < kNumberCameras; i++) {
            cameras[i] = new PhotonCamera(kCameraNames[i]);
            photonPoseEstimators[i] = new PhotonPoseEstimator(
                kAprilTagFieldLayout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,  
                kRobotToCameraTransforms[i]
            );   

            fields[i] = new Field2d();
            /*cameras are 0 through n-1*/
        }
        
        this.drivetrain = drivetrain;
    }
    
    private Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation) {//TODO review this method
        double smallestDistance = Double.POSITIVE_INFINITY;
        for (var target : estimation.targetsUsed) {
        var t3d = target.getBestCameraToTarget();
        var distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
        if (distance < smallestDistance)
            smallestDistance = distance;
        }
        double poseAmbiguityFactor = estimation.targetsUsed.size() != 1
            ? 1
            : Math.max(
                1,
                (estimation.targetsUsed.get(0).getPoseAmbiguity()
                    + kPoseAmbiguityOffset)
                    * kPoseAmbiguityMultiplier);
        //if there is 1 
        double confidenceMultiplier = Math.max(
            1,
            (Math.max(
                1,
                Math.max(0, smallestDistance - kNoisyDistanceMeters)
                    * kDistanceWeight)
                * poseAmbiguityFactor)
                / (1
                    + ((estimation.targetsUsed.size() - 1) * kTagPresenceWeight)));

        return kVisionStandardDeviations.times(confidenceMultiplier);
    }

    /** Gets the unread results for each camera, then add each result to its own photonPoseEstimator, 
        and add the result of the pose estimator to the swerve one.
        Call this method once every loop */
    public void update() {
        for(int i = 0; i < kNumberCameras; i++) {
            PhotonCamera currentCamera = cameras[i];

            var results = currentCamera.getAllUnreadResults();

            for(PhotonPipelineResult result : results) {
                Optional<EstimatedRobotPose> optionalEstimatedPose = photonPoseEstimators[i].update(
                    result,
                    currentCamera.getCameraMatrix(),
                    currentCamera.getDistCoeffs()
                 );

                if (optionalEstimatedPose.isPresent()) {
                    EstimatedRobotPose estimatedPose = optionalEstimatedPose.get();        
                    
                    drivetrain.addVisionMeasurement(
                        estimatedPose.estimatedPose.toPose2d(), 
                        estimatedPose.timestampSeconds, 
                        confidenceCalculator(estimatedPose)
                    );

                    fields[i].setRobotPose(estimatedPose.estimatedPose.toPose2d());
                }
            }
        }
    }
}
   