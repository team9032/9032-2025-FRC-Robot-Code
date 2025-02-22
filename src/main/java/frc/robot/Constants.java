package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.localization.CameraConstants;
import frc.robot.subsystems.swerve.SwerveConstants;

public final class Constants {
    public static class DriverConstants {
        public static final boolean kRunSysId = false;

        public static final int kDriveControllerPort = 0;

        public static final double kMaxSpeed = SwerveConstants.kSpeedAt12Volts.magnitude();
        public static final double kRotationRate = 4 * Math.PI;

        public final static FieldCentric kDriveRequest = new FieldCentric()
            .withDeadband(kMaxSpeed * 0.05) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    }

    public static class PathplannerConfig {
        public static final PIDConstants kTranslationPID = new PIDConstants(5.0);//TODO Tune
        public static final PIDConstants kRotationPID = new PIDConstants(5.0);

        public static final ApplyRobotSpeeds kPathPlannerDriveRequest = new ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

        public static final PathConstraints kdynamicPathConstraints = new PathConstraints(
            2,//TODO change these
            4, 
            Math.PI, 
            2 * Math.PI
        );
    }

    public static final class ObjectAimingConstants {
        public static final String kObjectTrackingCameraName = "FrontCenterCamera";

        public static final int kCycleAmtSinceTargetSeenCutoff = 10;
        public static final double kPitchDifferenceCutoff = 2.0;

        public static final double kRotationSetpoint = 18.8;
        //TODO make faster
        public static final double kDrivingSpeed = 2.0;//Meters per second

        /* PID Constants */
        public static final double kPRotation = 0.15;
        public static final double kDRotation = 0.002;

        /* Class Ids */
        public static final int kCoralId = 1;
        public static final int kAlgaeId = 0;
    }

    public static final class LocalizationConstants {//TODO all under need to be tuned 
        /* Constants for the confidence calculator */
        public static final double kPoseAmbiguityOffset = 0.2;
        public static final double kPoseAmbiguityMultiplier = 4;
        public static final double kNoisyDistanceMeters = 2.5;
        public static final double kDistanceWeight = 7;
        public static final int kTagPresenceWeight = 10;

        public static final Matrix<N3, N1> kBaseStandardDeviations = VecBuilder.fill(
            1,//X
            1,//Y
            1 * Math.PI//Theta
        );

        public static final String kAprilTagFieldLayoutName = "2025-reefscape.json";//Loads from a JSON file in deploy

        public static final CameraConstants[] kCameraConstants = new CameraConstants[] {
            new CameraConstants("FrontCenterCamera", new Transform3d(
                new Translation3d(Units.inchesToMeters(15.5), 0, Units.inchesToMeters(14.0)), 
                new Rotation3d(0, 0, 0)),
                true
            ),
            new CameraConstants("LeftCamera", new Transform3d(
                new Translation3d(Units.inchesToMeters(-0.75), Units.inchesToMeters(15.5), Units.inchesToMeters(16.0)), 
                new Rotation3d(0, 0, Math.PI / 2.0)),
                false
            ),
            new CameraConstants("BackCamera", new Transform3d(
                new Translation3d(-Units.inchesToMeters(15.5), 0, Units.inchesToMeters(14.25)), 
                new Rotation3d(0, 0, Math.PI)),
                false
            ),
            new CameraConstants("RightCamera", new Transform3d(
                new Translation3d(0.0, Units.inchesToMeters(14.75), Units.inchesToMeters(13.5)), 
                new Rotation3d(0, 0, -Math.PI / 2.0))
                ,false
            ),
        };
    }
}
