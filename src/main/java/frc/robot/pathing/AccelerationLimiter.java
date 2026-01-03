package frc.robot.pathing;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.GeometryUtil;

public class AccelerationLimiter {
    private Translation2d previousVelocity;
    private double previousTime;

    private Translation2d previousAcceleration;
    private boolean limitedTangentialAccel = false;
    private boolean limitedCentripetalAccel = false;

    public AccelerationLimiter() {}

    public void reset(Translation2d initialVelocity) {
        previousVelocity = initialVelocity;
        previousTime = Utils.getCurrentTimeSeconds();
    }   

    /** Limits the target velocity based on the acceleration component limits. Call this every loop cycle. */
    public Translation2d applyLimits(Translation2d targetVelocity, double maxForwardTangentialAcceleration, double maxCentripetalAcceleration) {
        /* Find target acceleration */
        double currentTime = Utils.getCurrentTimeSeconds();
        double dt = currentTime - previousTime;
        var targetAcceleration = (targetVelocity.minus(previousVelocity)).div(dt);

        /* Break acceleration into components in the direction of and orthogonal to velocity */
        Translation2d tangentialAcceleration = GeometryUtil.project(targetAcceleration, targetVelocity);
        Translation2d centripetalAcceleration = targetAcceleration.minus(tangentialAcceleration);

        /* Only limit tangential acceleration if it will cause forward acceleration and not deceleration */
        if (GeometryUtil.dotProduct(targetAcceleration, targetVelocity) > 0) {
            limitedTangentialAccel = tangentialAcceleration.getNorm() > maxForwardTangentialAcceleration;
            tangentialAcceleration = GeometryUtil.limitMagnitude(tangentialAcceleration, maxForwardTangentialAcceleration);
        } 
        else
            limitedTangentialAccel = false;

        /* Always limit centripetal acceleration */
        limitedCentripetalAccel = centripetalAcceleration.getNorm() > maxCentripetalAcceleration;
        centripetalAcceleration = GeometryUtil.limitMagnitude(centripetalAcceleration, maxCentripetalAcceleration);

        /* Go from acceleration components to target acceleration */
        targetAcceleration = tangentialAcceleration.plus(centripetalAcceleration);

        /* Integrate acceleration to find velocity */
        targetVelocity = previousVelocity.plus(targetAcceleration.times(dt));

        /* Store previous values */
        previousVelocity = targetVelocity;
        previousTime = currentTime;
        previousAcceleration = targetAcceleration;

        return targetVelocity;
    }

    public void publishLimitingStatus(String prefix) {
        SmartDashboard.putBoolean(prefix + "/Tangential Acceleration Limited", limitedTangentialAccel);
        SmartDashboard.putBoolean(prefix + "/Centripetal Acceleration Limited", limitedCentripetalAccel);
        SmartDashboard.putNumber(prefix + "/Limited Acceleration Magnitude", previousAcceleration.getNorm());
    }
}
