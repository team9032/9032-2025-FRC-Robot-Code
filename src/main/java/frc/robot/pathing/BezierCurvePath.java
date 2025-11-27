package frc.robot.pathing;

import java.util.ArrayList;
import java.util.List;
import java.util.TreeMap;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class BezierCurvePath {
    private final List<BezierCurve> curves = new ArrayList<>();

    private final InterpolatingDoubleTreeMap distanceToXLookupTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap distanceToYLookupTable = new InterpolatingDoubleTreeMap();
    private final TreeMap<Double, Translation2d> distanceToDirectionMap = new TreeMap<>();

    private final double length;

    private final double finalSpeed;

    public BezierCurvePath(List<BezierCurvePoint> points, double finalSpeed) {
        if (points.size() < 2)
            throw new IllegalArgumentException("2 or more points are needed");

        double totalDistance = 0.0;
        Translation2d previousCurveEndTranslation = null;
        /* Creates n - 1 curves for n points */
        for (int i = 0; i < points.size() - 1; i++) {
            /* Creates a curve that joins the current and the next point */
            var curve = new BezierCurve(points.get(i), points.get(i + 1), distanceToXLookupTable, distanceToYLookupTable, totalDistance, distanceToDirectionMap, previousCurveEndTranslation);
            curves.add(curve);

            totalDistance += curve.getLength();

            previousCurveEndTranslation = curve.getEndPoint().targetPose().getTranslation();
        }

        length = totalDistance;

        this.finalSpeed = finalSpeed;
    }

    public Translation2d samplePathPosition(double distance) {
        if (distance > length)
            throw new IllegalArgumentException("Distance must be less than " + length);

        return new Translation2d(distanceToXLookupTable.get(distance), distanceToYLookupTable.get(distance));
    }

    public Translation2d samplePathDirection(double distance) {
        if (distance > length)
            throw new IllegalArgumentException("Distance must be less than " + length);

        return distanceToDirectionMap.ceilingEntry(distance).getValue();
    }

    public double getLength() {
        return length;
    }

    public double getFinalSpeed() {
        return finalSpeed;
    }

    public Rotation2d getInitialRotation() {
        return curves.get(0).getStartPoint().targetPose().getRotation();
    }

    public Rotation2d getFinalRotation() {
        return curves.get(curves.size() - 1).getEndPoint().targetPose().getRotation();
    }
}