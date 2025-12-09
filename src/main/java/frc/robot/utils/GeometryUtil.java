package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;

public class GeometryUtil {
    public static Translation2d normalize(Translation2d translation) {
        double magnitude = translation.getNorm();

        return magnitude > 1e-6 ? translation.div(magnitude) : Translation2d.kZero;
    }
}
