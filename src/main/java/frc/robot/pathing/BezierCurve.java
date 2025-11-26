package frc.robot.pathing;

import java.util.TreeMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import static frc.robot.pathing.PathingConstants.kCurveSampleAmount;;

public class BezierCurve {
    private final BezierCurvePoint startPoint;
    private final BezierCurvePoint endPoint;

    private final double length;

    public BezierCurve(BezierCurvePoint startPoint, BezierCurvePoint endPoint, InterpolatingDoubleTreeMap xLookupTable, InterpolatingDoubleTreeMap yLookupTable, InterpolatingDoubleTreeMap distanceToTimeLookupTable, int timeOffset, double distanceOffset, TreeMap<Double, Translation2d> distanceToDirectionMap) {
        this.startPoint = startPoint;
        this.endPoint = endPoint;

        double totalDistance = distanceOffset;
        var previousTranslation = Translation2d.kZero;
        for (int i = 0; i <= kCurveSampleAmount; i++) {
            double time = (i / kCurveSampleAmount) + timeOffset;

            double x = sampleRawCurveX(time);
            double y = sampleRawCurveY(time);

            xLookupTable.put(time, x);
            yLookupTable.put(time, y);

            totalDistance += Math.hypot(x, y);

            distanceToTimeLookupTable.put(totalDistance, time);

            var currentTranslation = new Translation2d(x, y);
            if (previousTranslation != Translation2d.kZero) {
                var direction = currentTranslation.minus(previousTranslation);

                distanceToDirectionMap.put(totalDistance, direction);
            }

            previousTranslation = currentTranslation;
        }

        length = totalDistance;
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
        double x = (Math.pow(1 - time, 3) * startPoint.location().getX()) + 
            (3 * time * Math.pow(1 - time, 2) * startPoint.nextControlPoint().getX()) +
            (3 * time * time * (1 - time) * endPoint.previousControlPoint().getX()) +
            (Math.pow(time, 3) * endPoint.location().getX());

        return x;
    }

    private double sampleRawCurveY(double time) {
        double y = (Math.pow(1 - time, 3) * startPoint.location().getY()) + 
            (3 * time * Math.pow(1 - time, 2) * startPoint.nextControlPoint().getY()) +
            (3 * time * time * (1 - time) * endPoint.previousControlPoint().getY()) +
            (Math.pow(time, 3) * endPoint.location().getY());

        return y;
    }
}