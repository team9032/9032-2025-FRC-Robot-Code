package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.SwerveConstants;

public final class Constants {
    public static class DriverConstants {
        public static final int kDriveControllerPort = 0;

        public static final double kMaxSpeed = SwerveConstants.kSpeedAt12Volts.magnitude();
        public static final double kRotationRate = 4 * Math.PI;

        public final static FieldCentric kDriveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(0.1) // TODO tune
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    }

    public static class PathplannerConfig {
        private static final ModuleConfig kModuleConfig = new ModuleConfig(//TODO Find the actual constants
            SwerveConstants.kWheelRadius.baseUnitMagnitude(), 
            5.0, 
            1.0, 
            DCMotor.getKrakenX60(1), 
            80.0, 
            1
        );

        public static final RobotConfig kRobotConfig = new RobotConfig(//TODO Find the actual constants
            8.0, 
            8.0, 
            kModuleConfig, 
            Units.inchesToMeters(24.0),
            Units.inchesToMeters(24.0)
        );

        public static final PIDConstants kTranslationPID = new PIDConstants(0.0);//TODO Tune
        public static final PIDConstants kRotationPID = new PIDConstants(0.0);
    }
}
