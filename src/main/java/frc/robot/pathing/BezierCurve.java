package frc.robot.pathing;

import java.util.TreeMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import static frc.robot.pathing.PathingConstants.kCurveSampleAmount;;

public class BezierCurve {
    private final BezierCurvePoint startPoint;
    private final BezierCurvePoint endPoint;

    private final double length;

    public BezierCurve(BezierCurvePoint startPoint, BezierCurvePoint endPoint, InterpolatingDoubleTreeMap xLookupTable, InterpolatingDoubleTreeMap yLookupTable, double distanceOffset, TreeMap<Double, Translation2d> distanceToDirectionMap, Translation2d previousCurveEndTranslation) {
        this.startPoint = startPoint;
        this.endPoint = endPoint;

        double totalDistance = distanceOffset;
        var previousTranslation = previousCurveEndTranslation;
        for (int i = 0; i <= kCurveSampleAmount; i++) {
            double time = (double) i / kCurveSampleAmount;

            /* Find x and y coordinates by sampling the curve */
            double x = sampleRawCurveX(time);
            double y = sampleRawCurveY(time);
            var currentTranslation = new Translation2d(x, y);

            if (previousTranslation != null) {
                /* Find distance by approximating the path as line segments */
                totalDistance += previousTranslation.getDistance(currentTranslation);

                /* Find the path direction by subtracting the previous point from the current point */
                var direction = currentTranslation.minus(previousTranslation);
                distanceToDirectionMap.put(totalDistance, direction);
            }

            xLookupTable.put(totalDistance, x);
            yLookupTable.put(totalDistance, y);

            previousTranslation = currentTranslation;
        }

        length = totalDistance - distanceOffset;
    }

    public double getLength() {
        return length;
    }

    public BezierCurvePoint getStartPoint() {
        return startPoint;
    }

    public BezierCurvePoint getEndPoint() {
        return endPoint;
    }

    private double sampleRawCurveX(double time) {
        double x = (Math.pow(1 - time, 3) * startPoint.targetPose().getX()) + 
            (3 * time * Math.pow(1 - time, 2) * startPoint.nextControlPoint().getX()) +
            (3 * time * time * (1 - time) * endPoint.previousControlPoint().getX()) +
            (Math.pow(time, 3) * endPoint.targetPose().getX());

        return x;
    }

    private double sampleRawCurveY(double time) {
        double y = (Math.pow(1 - time, 3) * startPoint.targetPose().getY()) + 
            (3 * time * Math.pow(1 - time, 2) * startPoint.nextControlPoint().getY()) +
            (3 * time * time * (1 - time) * endPoint.previousControlPoint().getY()) +
            (Math.pow(time, 3) * endPoint.targetPose().getY());

        return y;
    }
}