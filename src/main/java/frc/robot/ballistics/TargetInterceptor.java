package frc.robot.ballistics;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.utils.GeometryUtil;

public class TargetInterceptor {
   private static final double g = 9.81;

   public static InterceptorResult interceptTargetWithParabolicTrajectory(Pose3d shooterPose, Pose3d targetPose, double shootVelocity, Translation2d robotVelocity) {
        var targetTranslation = targetPose.getTranslation().toTranslation2d();
        var shooterTranslation = shooterPose.getTranslation().toTranslation2d();
        
        double heightDifference = targetPose.getZ() - shooterPose.getZ();
        double groundDistanceToTarget = targetTranslation.getDistance(shooterTranslation);

        /* Use parabolic projectile motion equation to find the launch pitch at standstill */
        double vsquared = Math.pow(shootVelocity, 2);
        double launchPitch = Math.atan(
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
        double launchYaw = targetTranslation.minus(shooterTranslation).getAngle().getRadians();

        /* Create a 3d vector that hits the target at standstill */
        Translation3d launchVector = new Translation3d(shootVelocity, new Rotation3d(0.0, launchPitch, launchYaw));

        /* Subtract robot velocity components from the launch vector so the vector will still intercept while the robot is moving */
        launchVector = launchVector.minus(new Translation3d(robotVelocity));

        /* Find the pitch and yaw from the 3d launch vector */
        var launchVector2d = launchVector.toTranslation2d();
        double shooterPitch = Math.atan2(launchVector.getZ(), launchVector2d.getNorm());
        double shooterYaw = launchVector2d.getAngle().getRadians();

        double newShootVelocity = launchVector.getNorm();

        return new InterceptorResult(shooterPitch, shooterYaw, newShootVelocity);
   }

   public static InterceptorResult interceptTargetWithSimulatedTrajectory(Pose3d shooterPose, Pose3d targetPose, double shootVelocity, Translation2d robotVelocity) {
    var targetTranslation = targetPose.getTranslation().toTranslation2d();
    var shooterTranslation = shooterPose.getTranslation().toTranslation2d();
    
    /* Point the shooter at the target */
    var vectorToTarget = targetTranslation.minus(shooterTranslation);
    double launchYaw = vectorToTarget.getAngle().getRadians();

    /* Break robot velocity into components */
    var velocityInDirectionOfTarget = GeometryUtil.project(robotVelocity, vectorToTarget);
    var perpendicularVelocity = robotVelocity.minus(velocityInDirectionOfTarget);

    /* Use the best fit cubic polynomial from simulation to find launch pitch from distance and velocity in the direction of the target */
    double groundDistanceToTarget = targetTranslation.getDistance(shooterTranslation);
    double vectorPitch = groundDistanceToTarget + velocityInDirectionOfTarget.getNorm();//TODO this will be a function from simulation

    /* Create a 3d vector that hits the target accounting for velocity in the direction of the target */
    Translation3d launchVector = new Translation3d(shootVelocity, new Rotation3d(0.0, vectorPitch, launchYaw));

    /* Subtract perpendicular robot velocity component from the launch vector so the vector will still intercept while the robot is moving */
    launchVector = launchVector.minus(new Translation3d(perpendicularVelocity));

    /* Find the pitch and yaw from the 3d launch vector */
    var launchVector2d = launchVector.toTranslation2d();
    double shooterPitch = Math.atan2(launchVector.getZ(), launchVector2d.getNorm());
    double shooterYaw = launchVector2d.getAngle().getRadians();

    double newShootVelocity = launchVector.getNorm();

    return new InterceptorResult(shooterPitch, shooterYaw, newShootVelocity);
}
}
