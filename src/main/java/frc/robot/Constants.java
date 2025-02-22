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
            .withDeadband(kMaxSpeed * 0.01) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    }

    public static class PathplannerConfig {
        public static final PIDConstants kTranslationPID = new PIDConstants(5.0);//TODO Tune
        public static final PIDConstants kRotationPID = new PIDConstants(5.0);

        public static final ApplyRobotSpeeds kClosedLoopDriveRequest = new ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

        public static final PathConstraints kDynamicPathConstraints = new PathConstraints(
            1,//TODO change these
            2, 
            1 * Math.PI, 
            2 * Math.PI
        );

        public static final double kAlignmentXYkP = 2;//TODO Tune better
        public static final double kAlignmentXYkD = 0;
        
        public static final double kAlignmentRotkP = 10.0;
        public static final double kAlignmentRotkD = 0;

        public static final double kXYAlignmentTolerance = Units.inchesToMeters(0.5);
        public static final double kRotAlignmentTolerance = Units.degreesToRadians(1);
    }

    public static final class LocalizationConstants {//TODO all under need to be tuned 
        /* Constants for the confidence calculator */
        public static final double kPoseAmbiguityMultiplier = 40;
        public static final double kNoisyDistanceMeters = 2.5;
        public static final double kDistanceWeight = 7;

        /* Thresholds for when to reject an estimate */
        public static final double kAmbiguityThreshold = 0.2;
        public static final double kDistanceThreshold = 4.0;//Meters
        
        public static final Matrix<N3, N1> kSingleTagBaseStandardDeviations = VecBuilder.fill(
            1,//X
            1,//Y
            1 * Math.PI//Theta
        );

        public static final Matrix<N3, N1> kMultiTagBaseStandardDeviations = VecBuilder.fill(
            0.25,//X
            0.25,//Y
            0.5 * Math.PI//Theta
        );

        public static final String kAprilTagFieldLayoutName = "2025-reefscape.json";//Loads from a JSON file in deploy

        /* Used for cameras mounted on swerve modules */
        public static final double kCameraHeight = Units.inchesToMeters(8.0);//Height off of the ground in meters
        public static final double kCameraPitch = Units.degreesToRadians(17.0);//The camera's pitch

        public static final double kXandYCoord = 0.282575;

        public static final CameraConstants[] kCameraConstants = new CameraConstants[] {
            new CameraConstants("FrontRightCamera", new Transform3d(
                new Translation3d(kXandYCoord, -kXandYCoord, kCameraHeight), 
                new Rotation3d(0, kCameraPitch,  -(Math.PI / 4.0)))
            ),
            new CameraConstants("FrontLeftCamera", new Transform3d(
                new Translation3d(kXandYCoord, kXandYCoord, kCameraHeight), 
                new Rotation3d(0, kCameraPitch, (Math.PI / 4.0)))
            ),
            new CameraConstants("BackCenterCamera", new Transform3d(
                    new Translation3d(-0.073025,0.041275,0.25146),
                    new Rotation3d(0,0,Math.PI))
            ),
            new CameraConstants("BackRightCamera", new Transform3d(
                new Translation3d(0.104775,-0.352425,0.2794),
                new Rotation3d(0,0,Math.PI))
            ),
            new CameraConstants("BackLeftCamera", new Transform3d(
                new Translation3d(0.104775,0.352425,0.35814),
                new Rotation3d(0,0,Math.PI))
            )
        };
    }
}
