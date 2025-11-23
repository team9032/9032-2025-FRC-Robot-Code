package frc.robot.pathing;

import edu.wpi.first.math.geometry.Translation2d;

public record BezierCurve(BezierCurvePoint startPoint, BezierCurvePoint endPoint) {
    public Translation2d sampleCurve(double time) {
        double x = (Math.pow(1 - time, 3) * startPoint.location().getX()) + 
            (3 * time * Math.pow(1 - time, 2) * startPoint.nextControlPoint().getX()) +
            (3 * time * time * (1 - time) * endPoint.previousControlPoint().getX()) +
            (Math.pow(time, 3) * endPoint.location().getX());

        double y = (Math.pow(1 - time, 3) * startPoint.location().getY()) + 
            (3 * time * Math.pow(1 - time, 2) * startPoint.nextControlPoint().getY()) +
            (3 * time * time * (1 - time) * endPoint.previousControlPoint().getY()) +
            (Math.pow(time, 3) * endPoint.location().getY());

        return new Translation2d(x, y);
    }
}