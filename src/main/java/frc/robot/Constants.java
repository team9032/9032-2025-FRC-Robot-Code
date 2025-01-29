package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.Idle;
import com.pathplanner.lib.config.PIDConstants;
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
    }

    public static class ElevatorConfigs {
        private static final MotionMagicConfigs elevatorMotionMagicConfig = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(5)
            .withMotionMagicAcceleration(10)
            .withMotionMagicJerk(100);

        private static final Slot0Configs elevatorPIDConfig = new Slot0Configs()
            .withKP(1)
            .withKI(0)
            .withKD(0)
            .withKG(0);
        //FIXME hi HARSHIL PANDENATOR

        public static final CurrentLimitsConfigs kElevatorCurrentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimit(120);

        public static final TalonFXConfiguration kELevatorMotorConfig = new TalonFXConfiguration()
            .withMotionMagic(elevatorMotionMagicConfig)
            .withSlot0(elevatorPIDConfig)
            .withCurrentLimits(kElevatorCurrentLimits);

        public static final double kElevatorDownPos = 0;
        public static final double kElevatorMidLow = 22.5;
        public static final double kElevatorMidHigh = 45;
        public static final double kElevatorMax = 90;
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

        public static final double kRollerMotorPower = 0.5;

        public static final CurrentLimitsConfigs rollerMotorCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimit(120);

        public static final TalonFXConfiguration kRollerMotorConfiguration= new TalonFXConfiguration()
            .withCurrentLimits(rollerMotorCurrentLimitConfigs);
    }

    public static final class IntakeConstants {
        public static final CurrentLimitsConfigs kArmMotorCurrentLimit = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimit(120);

        public static final Slot0Configs kArmMotorPidConstants = new Slot0Configs()
            .withKP(0.17)
            .withKD(0)
            .withKG(0) // TODO tune values
            .withKS(0)
            .withKV(.117)
            .withKA(0);

        public static final MotionMagicConfigs kArmMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(120)
            .withMotionMagicAcceleration(80); // TODO tune values

        public static final TalonFXConfiguration kArmMotorConfig = new TalonFXConfiguration()
            .withCurrentLimits(kArmMotorCurrentLimit)
            .withSlot0(kArmMotorPidConstants)
            .withMotionMagic(kArmMotionMagicConfigs);

        public static final TalonFXConfiguration kWheelMotorConfig = new TalonFXConfiguration();
        public static final int kExtensionMotorID = 35;
        public static final int kWheelMotorID = 36;

        public static final double kEjectPower = -1.0;
        public static final double kGroundPosition = 28828;
        public static final double kIntakePower = 1.0;
        public static final double kStowPosition = 1000;
    }

    public static class ArmConstants {
        public static final int kArmMotorId = 25;
        public static final double kArmTroughPos = 0.05;
        public static final double kArmLevel1Pos = 0.1; //TODO change
        public static final double kArmLevel2Pos = 0.15;
        public static final double kArmLevel3Pos = 0.2;
        public static final boolean kArmMotorInverted = false;

        public static final CurrentLimitsConfigs kArmMotorCurrentLimit = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimit(120);

        public static final Slot0Configs kArmMotorPidConstants = new Slot0Configs() //TODO change all to neo stuff
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
        public static final int kEndEffectorID = 26;
        public static final double kEndEffectorOuttakeSpeed = 0.25;
        public static final double kEndEffectorIntakeSpeed = -0.5;
        public static final int kEndEffectorBeamBreakID = 27;
        public static final Slot0Configs kEndEffectorPID = new Slot0Configs()
            .withKA(0) //TODO tune
            .withKD(0)
            .withKP(1)
            .withKV(0)
            .withKS(0);

        public static final CurrentLimitsConfigs kCurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(120)
            .withSupplyCurrentLimit(40);

        public static final MotorOutputConfigs kEndEffectorOutputConfigs = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast);

        public static final TalonFXConfiguration kEndEffectorConfig = new TalonFXConfiguration()
            .withCurrentLimits(kCurrentLimits)
            .withSlot0(kEndEffectorPID)
            .withMotorOutput(kEndEffectorOutputConfigs);
    }
}