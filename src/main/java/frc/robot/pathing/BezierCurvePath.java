package frc.robot.pathing;

import java.util.ArrayList;
import java.util.List;
import java.util.TreeMap;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class BezierCurvePath {
    private final List<BezierCurve> curves = new ArrayList<>();

    private final InterpolatingDoubleTreeMap xLookupTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap yLookupTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap distanceToTimeLookupTable = new InterpolatingDoubleTreeMap();
    private final TreeMap<Double, Translation2d> distanceToDirectionMap = new TreeMap<>();

    private final double length;

    private final double finalSpeed;

    public BezierCurvePath(List<BezierCurvePoint> points, double finalSpeed) {
        if (points.size() < 2)
            throw new IllegalArgumentException("2 or more points are needed");

        int curveAmount = points.size() - 1;

        double totalDistance = 0.0;
        Translation2d previousCurveEndTranslation = Translation2d.kZero;
        for (int i = 0; i < curveAmount; i++) {
            var curve = new BezierCurve(points.get(i), points.get(i + 1), xLookupTable, yLookupTable, distanceToTimeLookupTable, i, totalDistance, distanceToDirectionMap, previousCurveEndTranslation);

            curves.add(curve);

            totalDistance += curve.getLength();

            previousCurveEndTranslation = curve.getEndPoint().targetPose().getTranslation();
        }

        length = totalDistance;

        this.finalSpeed = finalSpeed;
    }

    public Translation2d samplePathPosition(double time) {
        if (time > curves.size())
            throw new IllegalArgumentException("Time must be less than " + curves.size());

        return new Translation2d(xLookupTable.get(time), yLookupTable.get(time));
    }

    public Translation2d samplePathDirection(double distance) {
        if (distance > length)
            throw new IllegalArgumentException("Distance must be less than " + length);

        System.out.println("dd " + distance);//TODO fix direction lookups
        if (distanceToDirectionMap.ceilingEntry(distance) == null) {
            System.out.println("d " + distance + " total " + length + " c1 " + curves.get(0).getLength());
            return new Translation2d();
        }
        return distanceToDirectionMap.ceilingEntry(distance).getValue();
    }

    public double getTimeFromDistance(double distance) {
        return distanceToTimeLookupTable.get(distance);
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