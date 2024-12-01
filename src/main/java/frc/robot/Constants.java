package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.SwerveConstants;

public final class Constants {
    public static class DriverConstants {
        public static final int kDriveControllerPort = 0;

        public static final double kMaxSpeed = SwerveConstants.kSpeedAt12Volts.magnitude();
        public static final double kRotationRate = 4 * Math.PI;

        public final static FieldCentric kDriveRequest = new FieldCentric()
            .withDeadband(0.1) // TODO tune
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    }

    public static class PathplannerConfig {
        private static final ModuleConfig kModuleConfig = new ModuleConfig(
            SwerveConstants.kWheelRadius.baseUnitMagnitude(), 
            5.0,//TODO Find the actual constants
            1.0,//For Coulson wheels
            DCMotor.getKrakenX60(1), 
            80.0,//...
            1
        );

        public static final RobotConfig kRobotConfig = new RobotConfig(
            8.0,//TODO Find the actual constants 
            8.0,//... 
            kModuleConfig, 
            Units.inchesToMeters(24.0),
            Units.inchesToMeters(24.0)
        );

        public static final PIDConstants kTranslationPID = new PIDConstants(0.0);//TODO Tune
        public static final PIDConstants kRotationPID = new PIDConstants(0.0);

        public static final ApplyRobotSpeeds kPathPlannerDriveRequest = new ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);//TODO tune
    }
    public static final class LocalizationConstants{
        public static final int kNumberCameras = 4; /*!!!manually add ncameras cameras!!!*/

        public static final String[] kCameraNames = new String[kNumberCameras]; //TODO manually add ncameras cameras
        public static final Transform3d[] kRobotToCameraTransforms = new Transform3d[kNumberCameras];
        
        /* Indexed 0 to n-1 */
        static {
            kCameraNames[0] = "camera0";
            kCameraNames[1] = "camera1";
            kCameraNames[2] = "camera2";
            kCameraNames[3] = "camera3";
            
            kRobotToCameraTransforms[0] = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
            kRobotToCameraTransforms[1] = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
            kRobotToCameraTransforms[2] = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
            kRobotToCameraTransforms[3] = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
        }
        //TODO all under need to be tuned 
        public static final double kAmbiguityThreshold = 0.2;
        public static final double kPoseAmbiguityOffset = 0.2;
        public static final double kPoseAmbiguityMultiplier = 4;
        public static final double kNoisyDistanceMeters = 2.5;
        public static final double kDistanceWeight = 7;
        public static final int kTagPresenceWeight = 10;
        
        public static final Matrix<N3, N1> kVisionStandardDeviations = VecBuilder.fill(
            1, //x
            1, //y
            1 * Math.PI //theta
            //nums can't be < 1
        );

        public static final AprilTagFieldLayout kAprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    }
}
