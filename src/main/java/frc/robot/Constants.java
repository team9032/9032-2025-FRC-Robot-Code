package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
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

        public static final double kJoystickDeadband = 0.02;//For Xbox Controllers
        public static final DriveRequestType kDriveRequestType = DriveRequestType.OpenLoopVoltage;
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

        public static final ModuleRequest kModuleRequest = new ModuleRequest()
            .withSteerRequest(SteerRequestType.MotionMagicExpo);

        public static final double kMaxSteerVelocity = 4.0;//TODO find (rad/s)
    }
}
