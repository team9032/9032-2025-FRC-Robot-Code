package frc.robot.localization;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.ElasticUtil;

import static frc.robot.Constants.LocalizationConstants.*;
import static frc.robot.Constants.ObjectAimingConstants.*;
import java.util.List;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.ArrayList;

public class Localization {
    private final SwerveDrivetrain<?, ?, ?> drivetrain;

    private final Field2d field;

    private final List<LocalizationCamera> cameras = new ArrayList<>();

    public Localization(SwerveDrivetrain<?, ?, ?> drivetrain) {  
        this.drivetrain = drivetrain;

        field = new Field2d();

        try {
            AprilTagFieldLayout layout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/" + kAprilTagFieldLayoutName);

            for (int i = 0; i < kCameraConstants.length; i++) {
                cameras.add(new LocalizationCamera(kCameraConstants[i], layout)); 
            }
        } catch(Exception e) {
            ElasticUtil.sendError("Error opening AprilTag field layout", "Localization will commit die!");
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
    
    /** Gets object tracking results from a camera. If the camera is not in object tracking mode, this will be empty. */
    public List<PhotonPipelineResult> getObjectTrackingResults(String cameraName) {
        for(LocalizationCamera camera : cameras) {
            if(camera.getName().equals(cameraName)) 
                return camera.getObjectTrackingResults();
        }

        return List.of();
    }

    /** Given a target's pitch in radians, finds the distance from the target to the robot */
    public double findDistanceToTarget(String cameraName, double targetPitch, double targetHeight) {
        for(var constants : kCameraConstants) {
            if(constants.name().equals(cameraName)) {
                var transform = constants.robotToCameraTransform();

                return PhotonUtils.calculateDistanceToTargetMeters(transform.getZ(), targetHeight, transform.getRotation().getY(), targetPitch);
            }
        }

        return 0.0;
    }
    
    public boolean canPlaceByReefDetection(String cameraName, int level) {
        var results = new ArrayList<PhotonPipelineResult>(getObjectTrackingResults(cameraName));

        /*delete results with no objects detected */
        results.removeIf((result) -> !result.hasTargets());

        /*get latest result */
        PhotonPipelineResult latestResult = results.get(0);
        for(var result : results) {
            if(result.getTimestampSeconds() > latestResult.getTimestampSeconds())
                latestResult = result;
        }
        var targets = latestResult.getTargets();

        /*filter bad ones out, too small, too left, too right */
        for(PhotonTrackedTarget target : targets){
            var targetCorners = new ArrayList<TargetCorner>(target.getDetectedCorners());
            double area = target.getArea();
            double leftX = targetCorners.get(0).x;
            
            for(TargetCorner corner : targetCorners) {
                if(corner.x < leftX){
                    leftX = corner.x;
                }
            }
            if(target.getDetectedObjectClassID() == kCoralId) {
                if(area<kCoralAreaMinimumThreshold){
                    continue;
                }
                if(leftX < kCoralCornerMinimumCoord){
                    continue;
                }
                if(leftX > kCoralCornerMaximumCoord){
                    continue;
                }

            } else if(target.getDetectedObjectClassID() == kAlgaeId) {
                if(area<kAlgaeAreaMinimumThreshold){
                    continue;
                }
                if(leftX < kAlgaeCornerMinimumCoord){
                    continue;
                }
                if(leftX > kAlgaeCornerMaximumCoord){
                    continue;
                }
            }

            /*check if this target interferes */
            double minY = targetCorners.get(0).y;
            
            for(TargetCorner corner : targetCorners) {
                if(corner.y < minY){
                    minY = corner.y;
                }
            }
            if(target.getDetectedObjectClassID() == kAlgaeId) {
                //algae here, 
                if(minY > kMinAlgaeYForUpper) {
                    //means it is algae above l3, 
                    if(level == 3){
                        return false;
                    }
                } else{
                    //means it is algae on top of l2, touching l3
                    if(level == 2 || level == 3){
                        return false;
                    }
                }
            } else if(target.getDetectedObjectClassID() == kCoralId) {
                //coral 
                if(minY > kL4YCoordThreshold) {
                    if(level == 4){
                        return false;
                    }
                    continue;
                }
                if(minY > kL3YCoordThreshold) {
                    if(level == 3){
                        return false;
                    }
                    continue;
                }
                if(minY > kL2YCoordThreshold) {
                    if(level == 2){
                        return false;
                    }
                    continue;
                }
            }
        }
        return true;
    }
}
   