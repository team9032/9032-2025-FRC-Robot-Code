package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.localization.LocalizationCamera;
import frc.robot.subsystems.swerve.SwerveConstants;

public final class Constants {
    public static class DriverConstants {
        public static final int kDriveControllerPort = 0;

        public static final double kMaxSpeed = SwerveConstants.kSpeedAt12Volts.magnitude();
        public static final double kRotationRate = 4 * Math.PI;

        public final static FieldCentric kDriveRequest = new FieldCentric()
            .withDeadband(kMaxSpeed * 0.05) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    }

    public static class PathplannerConfig {
        private static final ModuleConfig kModuleConfig = new ModuleConfig(
            SwerveConstants.kWheelRadius.baseUnitMagnitude(), 
            5.0,//TODO Find the actual constants
            1.0,//For Coulson wheels
            DCMotor.getKrakenX60(1), 
            70.0,//Default from swerve project
            1
        );

        public static final double kTrackwidth = Units.inchesToMeters(24.0);

        public static final Translation2d kflModuleOffset = new Translation2d(kTrackwidth / 2.0, kTrackwidth / 2.0);
        public static final Translation2d kfrModuleOffset = new Translation2d(kTrackwidth / 2.0, -kTrackwidth / 2.0);
        public static final Translation2d kblModuleOffset = new Translation2d(-kTrackwidth / 2.0, kTrackwidth / 2.0);
        public static final Translation2d kbrModuleOffset = new Translation2d(-kTrackwidth / 2.0, -kTrackwidth / 2.0);

        public static final RobotConfig kRobotConfig = new RobotConfig(
            Units.lbsToKilograms(55.0),//TODO Find the actual constants 
            (1.0 / 12.0) * Units.lbsToKilograms(55.0) * (1152),//... 
            kModuleConfig, 
            kflModuleOffset, kfrModuleOffset, kblModuleOffset, kbrModuleOffset
        );

        public static final PIDConstants kTranslationPID = new PIDConstants(5.0);//TODO Tune
        public static final PIDConstants kRotationPID = new PIDConstants(5.0);

        public static final ApplyRobotSpeeds kPathPlannerDriveRequest = new ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    }

    public static final class LocalizationConstants {//TODO all under need to be tuned 
        public static final LocalizationCamera[] kCameras = new LocalizationCamera[] {
            new LocalizationCamera("FrontCamera", new Transform3d(
                new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0))),
                //36.6 cm 
            new LocalizationCamera("LeftCamera", new Transform3d(new 
                Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,Math.PI/2))),

            new LocalizationCamera("BackCamera", new Transform3d(
                new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,Math.PI))),

            new LocalizationCamera("RightCamera", new Transform3d(
                new Translation3d(0, 0.0, 0.5), new Rotation3d(0,0,-Math.PI))),
                //14.5 inches, 14.5 inches + 3 cm - cam extends, 29 cm up
        };
        //29x29 inches
        public static final double kPoseAmbiguityOffset = 0.2;
        public static final double kPoseAmbiguityMultiplier = 4;
        public static final double kNoisyDistanceMeters = 2.5;
        public static final double kDistanceWeight = 7;
        public static final int kTagPresenceWeight = 10;
        
        public static final Matrix<N3, N1> kVisionStandardDeviations = VecBuilder.fill(
            1,//X
            1,//Y
            1 * Math.PI//Theta
        );

        public static final List<AprilTag> kAprilTags = new ArrayList<>();      
        static { 
            //TODO add zs, 55 inches
            kAprilTags.add(new AprilTag(2, new Pose3d(0, 3.2004, 1.397, new Rotation3d(0, 0, 0))));
            kAprilTags.add(new AprilTag(3, new Pose3d(3.0861, 0, 1.397, new Rotation3d(0, 0, Math.PI / 2))));
            kAprilTags.add(new AprilTag(3, new Pose3d(3.6576, 0, 1.397, new Rotation3d(0, 0, Math.PI / 2))));
            kAprilTags.add(new AprilTag(6, new Pose3d(7.2898, 3.2004,1.397, new Rotation3d(0, 0, Math.PI))));
            kAprilTags.add(new AprilTag(8, new Pose3d(3.6576, 6.35, 1.397, new Rotation3d(0,0, -Math.PI / 2))));
        } 
        
        public static final double kFieldLength = 7.2898;//X-axis
        public static final double kFieldWidth = 6.35;

        public static final AprilTagFieldLayout kAprilTagFieldLayout = new AprilTagFieldLayout(kAprilTags, kFieldLength, kFieldWidth);
    }
}
