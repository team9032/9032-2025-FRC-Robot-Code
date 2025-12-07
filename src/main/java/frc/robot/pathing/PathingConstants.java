package frc.robot.pathing;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class PathingConstants {
    /* Constraints */
    public static final Constraints kRotationConstraints = new Constraints(3 * Math.PI, 4 * Math.PI);

    public static final double kMaxAcceleration = 4.0;
    public static final double kMaxSpeed = 4.0;

    public static final double kTorqueLimitedSpeedStart = 3.5;

    /* PID Constants */
    public static final double kRotationkP = 8.0;
    public static final double kRotationkD = 0.1;

    /* Tolerances */
    public static final Distance kXYAlignmentTolerance = Inches.of(0.33);
    public static final Angle kRotAlignmentTolerance = Degrees.of(4);
}