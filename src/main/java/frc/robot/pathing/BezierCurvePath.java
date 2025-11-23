package frc.robot.pathing;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;

public class BezierCurvePath {
    private final List<BezierCurve> curves = new ArrayList<>();

    public BezierCurvePath(BezierCurvePoint startPoint, BezierCurvePoint endPoint) {
        this(List.of(startPoint, endPoint));
    }

    public BezierCurvePath(BezierCurvePoint startPoint, BezierCurvePoint intermediatePoint, BezierCurvePoint endPoint) {
        this(List.of(startPoint, intermediatePoint, endPoint));
    }

    public BezierCurvePath(List<BezierCurvePoint> points) {
        if (points.size() < 2)
            throw new IllegalArgumentException("2 or more points are needed");

        int curveAmount = points.size() - 1;

        for (int i = 0; i < curveAmount; i++) {
            curves.add(new BezierCurve(points.get(i), points.get(i + 1)));
        }
    }

    public Translation2d samplePath(double time) {
        int index = (int) Math.floor(time);

        return curves.get(index).sampleCurve(time - index);
    }
}