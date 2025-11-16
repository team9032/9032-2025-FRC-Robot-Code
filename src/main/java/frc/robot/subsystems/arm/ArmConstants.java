package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class ArmConstants {
    public static final int kArmMotorId = 18;

    public static final int kArmEncoderId = 38;
    public static final MagnetSensorConfigs kArmEncoderConfig = new MagnetSensorConfigs()
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
        .withAbsoluteSensorDiscontinuityPoint(0.5)
        .withMagnetOffset(-0.1032);

    public static final double kArmStowPos = 0.2;
    public static final double kArmCradlePos = -0.25;
    public static final double kArmL2ScorePos = 0.05; 
    public static final double kArmL3ScorePos = 0.05;
    public static final double kArmL4ScorePos = 0.09;
    public static final double kArmHighAlgaePos = 0.0;
    public static final double kArmLowAlgaePos = 0.0;
    public static final double kArmProcessorPos = 0;
    public static final double kArmNetPos = 0.38;
    public static final double kArmAlgaeGroundPos = -0.1;
    public static final double kArmClimbPos = 0.09;
    public static final double kArmCoralPreparedToScorePos = 0.17;
    public static final double kArmLowL1PreparedToScorePos = -0.06;
    public static final double kArmHighL1PreparedToScorePos = -0.03;

    public static final double kArmOverCradlePos = 0.0;
    public static final double kArmPositionTolerance = 0.005;

    public static final CurrentLimitsConfigs kArmMotorCurrentLimit = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(40)
        .withStatorCurrentLimit(120);
    
    public static final GravityTypeValue kArmGravityType = GravityTypeValue.Arm_Cosine;

    public static final Slot0Configs kArmMotorPidConstants = new Slot0Configs()
        .withKG(0.3)
        .withKP(130)
        .withKD(0.1)
        .withKA(0.25)
        .withKV(4.5)
        .withGravityType(kArmGravityType);

    public static final MotionMagicConfigs kArmMotionMagicConfigs = new MotionMagicConfigs()
        .withMotionMagicExpo_kV(2.5)
        .withMotionMagicExpo_kA(1.5); 

    public static final FeedbackConfigs kArmFeedbackConfigs = new FeedbackConfigs()
        .withSensorToMechanismRatio(1.0)
        .withRotorToSensorRatio(162240.0 / 3456.0)
        .withFeedbackRemoteSensorID(kArmEncoderId)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder);

    public static final TalonFXConfiguration kArmMotorConstants = new TalonFXConfiguration()
        .withCurrentLimits(kArmMotorCurrentLimit)
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
        )
        .withSlot0(kArmMotorPidConstants)
        .withMotionMagic(kArmMotionMagicConfigs)
        .withFeedback(kArmFeedbackConfigs);
}