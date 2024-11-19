package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;

import frc.robot.subsystems.swerve.SwerveConstants;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

public final class Constants {
    public static class DriverConstants {
        public static final int kDriveControllerPort = 0;

        public static final double kMaxSpeed = SwerveConstants.kSpeedAt12VoltsMps;
        public static final double kRotationRate = 4 * Math.PI;

        public final static FieldCentric kDriveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(0.1) // TODO tune
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    }
}
