package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;

public class GeometryUtil {
    public static Translation2d normalize(Translation2d translation) {
        double magnitude = translation.getNorm();

        return magnitude > 1e-6 ? translation.div(magnitude) : Translation2d.kZero;
    }

    public static Translation2d project(Translation2d toProject, Translation2d projectedOnto) {
        double projectedOntoMagnitude = projectedOnto.getNorm();

        return projectedOntoMagnitude > 1e-6 ?
            normalize(projectedOnto).times(dotProduct(toProject, projectedOnto) / projectedOntoMagnitude)
            : Translation2d.kZero;
    }

    public static double dotProduct(Translation2d translation1, Translation2d translation2) {
        return (translation1.getX() * translation2.getX()) + (translation1.getY() * translation2.getY());
    }

    public static Translation2d limitMagnitude(Translation2d translation, double magnitudeLimit) {
        double magnitude = Math.min(translation.getNorm(), magnitudeLimit);

        return normalize(translation).times(magnitude);
    }
}
