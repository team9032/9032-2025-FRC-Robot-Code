package frc.robot.pathing;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class PathingConstants {
    public static final double kStraightDriveDistance = Units.inchesToMeters(12);

    /* Constraints */
    public static final Constraints kRotationConstraints = new Constraints(3 * Math.PI, 3 * Math.PI);

    public static final double kMaxAcceleration = 4.0;
    public static final double kTrueMaxAcceleration = 9.0;
    public static final double kMaxSpeed = 4.4;
    public static final double kTrueMaxSpeed = 4.6;

    public static final double kTorqueLimitedSpeedStart = 4.0;//TODO no idea how to find

    /* PID Constants */
    public static final double kRotationkP = 8.0;
    public static final double kRotationkD = 0.1;
    public static final double kTranslationkP = 7.0;//Only used for simple drive to pose
    public static final double kTranslationkD = 0.1;

    /* Tolerances */
    public static final Distance kXYAlignmentTolerance = Inches.of(0.33);
    public static final Distance kRoughXYAlignmentTolerance = Inches.of(6);
    public static final Angle kRotationAlignmentTolerance = Degrees.of(4);

    public static final double kAcceptableEndingSpeed = 0.25;

    /* Drive Requests */
    public static final FieldCentric kFieldCentricClosedLoopDriveRequest = new FieldCentric()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    public static final ApplyRobotSpeeds kRobotRelativeClosedLoopDriveRequest = new ApplyRobotSpeeds()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);
}