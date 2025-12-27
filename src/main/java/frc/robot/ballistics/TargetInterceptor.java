package frc.robot.ballistics;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class TargetInterceptor {
   private static final double g = 9.81;

   public static InterceptorResult interceptTargetWithParabolicTrajectory(Pose3d shooterPose, Pose3d targetPose, double shootVelocity, Translation2d robotVelocity) {
        var targetTranslation = targetPose.getTranslation().toTranslation2d();
        var shooterTranslation = shooterPose.getTranslation().toTranslation2d();
        
        double heightDifference = targetPose.getZ() - shooterPose.getZ();
        double groundDistanceToTarget = targetTranslation.getDistance(shooterTranslation);

        /* Use parabolic projectile motion equation to find the launch pitch at standstill */
        double vsquared = Math.pow(shootVelocity, 2);
        double vectorPitch = Math.atan(
            (vsquared - 
                Math.sqrt(
                    Math.pow(shootVelocity, 4) 
                    - (g 
                        * ((g * Math.pow(groundDistanceToTarget, 2)) + (2 * heightDifference * vsquared))
                    )
                )
            )
            / (g * groundDistanceToTarget)
        );

        /* Point the shooter at the target */
        double vectorYaw = targetTranslation.minus(shooterTranslation).getAngle().getRadians();

        /* Create a 3d vector that hits the target at standstill */
        Translation3d launchVector = new Translation3d(shootVelocity, new Rotation3d(0.0, vectorPitch, vectorYaw));

        /* Subtract robot velocity components from the launch vector so the vector will still intercept while the robot is moving */
        launchVector = launchVector.minus(new Translation3d(robotVelocity));

        /* Find the pitch and yaw from the 3d launch vector */
        var launchVector2d = launchVector.toTranslation2d();
        double shooterPitch = Math.atan2(launchVector.getZ(), launchVector2d.getNorm());
        double shooterYaw = launchVector2d.getAngle().getRadians();

        double newShootVelocity = launchVector.getNorm();

        return new InterceptorResult(shooterPitch, shooterYaw, newShootVelocity);
   }
}
