package frc.robot.pathing;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class PathingConstants {
    /* Path following constants */
    public static final double kTranslationKP = 0.0;//TODO need to tune
    public static final double kTranslationKD = 0.0;

    public static final double kRotationKP = 0.0;
    public static final double kRotationKD = 0.0;

    public static final Distance kTranslationTolerance = Inches.of(0.33);
    public static final Angle kRotationTolerance = Degrees.of(4);
    public static final double kAcceptableEndingVelocity = 0.25;

    /* Path generation constants */
    public static final double kCurveSampleAmount = 20;

    /* Trajectory generation constants */
    public static final Constraints kDefaultTranslationConstraints = new Constraints(4, 5);
    public static final Constraints kDefaultRotationConstraints = new Constraints(3 * Math.PI, 4 * Math.PI);
}
