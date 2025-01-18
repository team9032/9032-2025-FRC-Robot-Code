package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
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
                5.0, // TODO Find the actual constants
                1.0, // For Coulson wheels
                DCMotor.getKrakenX60(1),
                70.0, // Default from swerve project
                1);

        public static final double kTrackwidth = Units.inchesToMeters(24.0);

        public static final Translation2d kflModuleOffset = new Translation2d(kTrackwidth / 2.0, kTrackwidth / 2.0);
        public static final Translation2d kfrModuleOffset = new Translation2d(kTrackwidth / 2.0, -kTrackwidth / 2.0);
        public static final Translation2d kblModuleOffset = new Translation2d(-kTrackwidth / 2.0, kTrackwidth / 2.0);
        public static final Translation2d kbrModuleOffset = new Translation2d(-kTrackwidth / 2.0, -kTrackwidth / 2.0);

        public static final RobotConfig kRobotConfig = new RobotConfig(
                Units.lbsToKilograms(55.0), // TODO Find the actual constants
                (1.0 / 12.0) * Units.lbsToKilograms(55.0) * (1152), // ...
                kModuleConfig,
                kflModuleOffset, kfrModuleOffset, kblModuleOffset, kbrModuleOffset);

        public static final PIDConstants kTranslationPID = new PIDConstants(5.0);// TODO Tune
        public static final PIDConstants kRotationPID = new PIDConstants(5.0);

        public static final ApplyRobotSpeeds kPathPlannerDriveRequest = new ApplyRobotSpeeds()
                .withDriveRequestType(DriveRequestType.Velocity)
                .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    }

    public static class ElevatorConfigs {
        private static final MotionMagicConfigs elevatorMotionMagicConfig = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(1)
                .withMotionMagicAcceleration(3);

        private static final Slot0Configs elevatorPIDConfig = new Slot0Configs()
                .withKP(0)
                .withKI(0)
                .withKD(0)
                .withKG(0);
        // hi HARSHIL PANDENATOR

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
}
