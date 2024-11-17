// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class KrakenSwerveConstants {
    private static final Slot0Configs steerConfig = new Slot0Configs()
        .withKP(0).withKI(0).withKD(0)
        .withKV(0).withKA(0).withKS(0); //TODO Tune
    private static final Slot0Configs driveConfig = new Slot0Configs()
        .withKP(0).withKI(0).withKD(0)
        .withKV(0).withKA(0).withKS(0); //TODO Tune

    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    public static final double kSpeedAt12VoltsMps = 0; //TODO Change

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
        .withCANbusName("rio")
        .withPigeon2Id(0); //TODO change

    public static final SwerveModuleConstantsFactory constantFactory = new SwerveModuleConstantsFactory()
        .withDriveMotorGearRatio(6.75)
        .withSteerMotorGearRatio(150/7)
        .withWheelRadius(2)
        .withSteerMotorGains(steerConfig)
        .withDriveMotorGains(driveConfig)
        .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
        .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
        .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
        .withFeedbackSource(SteerFeedbackType.RemoteCANcoder)
        .withCouplingGearRatio(0) //TODO change
        .withSlipCurrent(90) //TODO tune
        .withSteerMotorInverted(true);

    public static final SwerveModuleConstants frontLeft = constantFactory.createModuleConstants(0,0,0,0,0,0,false);
    public static final SwerveModuleConstants frontRight = constantFactory.createModuleConstants(0,0,0,0,0,0,false);
    public static final SwerveModuleConstants backLeft = constantFactory.createModuleConstants(0,0,0,0,0,0,false);
    public static final SwerveModuleConstants backRight = constantFactory.createModuleConstants(0,0,0,0,0,0,false);
    //TODO Set ID's and module positions/offsets once chassis is built!

    public final static SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
        .withDeadband(0.1) //TODO tune
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);
}
}
