package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

    public static class ElevatorConfigs {
        public static int kMotor1ID = 25; // TODO change CAN stuff
        public static int kMotor2ID = 26; // TODO change CAN stuff

        private static final MotionMagicConfigs kElevatorMotionMagicConfig = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(5)
            .withMotionMagicAcceleration(10);

        private static final Slot0Configs kElevatorPIDConfig = new Slot0Configs()
            .withKP(1)
            .withKD(0)
            .withKV(Double.POSITIVE_INFINITY)
            .withKA(Double.NaN)
            .withKS(Double.NEGATIVE_INFINITY)
            .withKG(0);
        //FIXME hi HARSHIL PANDENATOR

        public static final CurrentLimitsConfigs kElevatorCurrentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimit(120);

        public static final TalonFXConfiguration kELevatorMotorConfig = new TalonFXConfiguration()
            .withMotionMagic(kElevatorMotionMagicConfig)
            .withSlot0(kElevatorPIDConfig)
            .withCurrentLimits(kElevatorCurrentLimits);

        public static final double kElevatorTolerance = 5;

        //TODO change all elevator positions once we can find them
        public static final double kElevatorTrough = 0;
        public static final double kElevatorL1 = 22.5;
        public static final double kElevatorL2 = 45;
        public static final double kElevatorL3 = 90;
        public static final double kElevatorLowAlgae = 0;
        public static final double kElevatorHighAlgae = 0;
        public static final double kElevatorIndexerPos = 90;
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
            new CameraConstants("FrontCamera", new Transform3d(
                new Translation3d(Units.inchesToMeters(15.5), 0, Units.inchesToMeters(14.0)), 
                new Rotation3d(0, 0, 0))
            ),
            new CameraConstants("LeftCamera", new Transform3d(
                new Translation3d(Units.inchesToMeters(-0.75), Units.inchesToMeters(15.5), Units.inchesToMeters(16.0)), 
                new Rotation3d(0, 0, Math.PI / 2.0))
            ),
            new CameraConstants("BackCamera", new Transform3d(
                new Translation3d(-Units.inchesToMeters(15.5), 0, Units.inchesToMeters(14.25)), 
                new Rotation3d(0, 0, Math.PI))
            ),
            new CameraConstants("RightCamera", new Transform3d(
                new Translation3d(0.0, Units.inchesToMeters(14.75), Units.inchesToMeters(13.5)), 
                new Rotation3d(0, 0, -Math.PI / 2.0))
            ),
        };
    }

    public static final class IndexerConstants {
        public static final int kRollerMotorID = 0;
        public static final int kSensorPort = 0;

        public static final double kHasCoralRange = 150;

        public static final double kRollerMotorPower = 0.5;

        public static final CurrentLimitsConfigs kRollerMotorCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimit(120);

        public static final TalonFXConfiguration kRollerMotorConfig = new TalonFXConfiguration()
            .withCurrentLimits(kRollerMotorCurrentLimitConfigs);
    }

    public static final class IntakeConstants {
        public static final CurrentLimitsConfigs kIntakeMotorCurrentLimit = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimit(120);

        public static final Slot0Configs kIntakeMotorPidConstants = new Slot0Configs()
            .withKP(0.17)
            .withKD(0)
            .withKG(0) // TODO tune values
            .withKS(0)
            .withKV(.117)
            .withKA(0);

        public static final MotionMagicConfigs kIntakeMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(120)
            .withMotionMagicAcceleration(80); // TODO tune values

        public static final TalonFXConfiguration kIntakeMotorConfig = new TalonFXConfiguration()
            .withCurrentLimits(kIntakeMotorCurrentLimit)
            .withSlot0(kIntakeMotorPidConstants)
            .withMotionMagic(kIntakeMotionMagicConfigs);

        public static final MotorOutputConfigs kWheelOutputConfigs = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast); //TODO change if bad

        public static final TalonFXConfiguration kWheelMotorConfig = new TalonFXConfiguration()
            .withCurrentLimits(kIntakeMotorCurrentLimit)
            .withMotorOutput(kWheelOutputConfigs);
        
        public static final int kIntakeMotorID = 35;
        public static final int kWheelMotorID = 36;
        public static final int kPhotoelectricSensorID = 29;

        public static final double kEjectPower = -1.0;
        public static final double kEjectDelay = 0.2;
        public static final double kGroundPosition = 28828;
        public static final double kIntakePower = 1.0;
        public static final double kStowPosition = 1000;
    }

    public static class ArmConstants {
        public static final int kArmMotorId = 25;

        public static final double kArmIndexerPos = 0.05;
        public static final double kArmTroughPos = 0.05;
        public static final double kArmLevel1Pos = 0.1; //TODO change
        public static final double kArmLevel2Pos = 0.15;
        public static final double kArmLevel3Pos = 0.2;
        public static final double kArmHighAlgaePos = 0.25;
        public static final double kArmLowAlgaePos = 0.125;

        public static final double kArmPositionTolerance = 0.05;

        public static final CurrentLimitsConfigs kArmMotorCurrentLimit = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimit(120);

        public static final Slot0Configs kArmMotorPidConstants = new Slot0Configs()
            .withKP(1)
            .withKD(0)
            .withKG(0) // TODO tune values
            .withKS(0)
            .withKV(0)
            .withKA(0);

        public static final MotionMagicConfigs kArmMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(20)
            .withMotionMagicAcceleration(40); // TODO tune values

        public static final TalonFXConfiguration kArmMotorConstants = new TalonFXConfiguration()
            .withCurrentLimits(kArmMotorCurrentLimit)
            .withSlot0(kArmMotorPidConstants)
            .withMotionMagic(kArmMotionMagicConfigs);
    }

    public static class ClimberConstants {
        public static final int kMotorID = 50;
        
        public static final CurrentLimitsConfigs kMotorCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(70)
            .withStatorCurrentLimit(120);

        public static final TalonFXConfiguration kMotorConfiguration= new TalonFXConfiguration()
            .withCurrentLimits(kMotorCurrentLimitConfigs); 
    }

    public static final class EndEffectorConstants {
        public static final int kMainEndEffectorID = 26;
        public static final int kPhotoelectricSensor1ID = 27;
        public static final int kSecondaryEndEffectorID = 28;
        public static final int kAlgaeDistSensorID = 29;
        public static final int kPhotoelectricSensor2ID = 30;

        public static final double kOuttakePower = 0.25;
        public static final double kIntakeFromSourcePower = -0.5;
        public static final double kReceiveFromIndexerPower = 0.25;
        public static final double kIntakeAlgaePower = 0.25;
        public static final double kSlowIntakeFromSourcePower = -0.25; //TODO tune
        public static final double kSlowReceiveFromIndexerPower = 0.1; //TODO tune

        public static final double kOuttakeWait = 0.5;
        public static final double kHasAlgaeDist = 100;

        public static final CurrentLimitsConfigs kCurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(120)
            .withSupplyCurrentLimit(40);

        public static final MotorOutputConfigs kEndEffectorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive); //TODO change if bad

        public static final TalonFXConfiguration kEndEffectorConfig = new TalonFXConfiguration()
            .withCurrentLimits(kCurrentLimits)
            .withMotorOutput(kEndEffectorOutputConfigs);
    }
}