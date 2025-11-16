package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorConstants {
    public static final int kFrontElevatorID = 14; 

    private static final MotionMagicConfigs kElevatorMotionMagicConfig = new MotionMagicConfigs()
        .withMotionMagicExpo_kV(0.2)
        .withMotionMagicExpo_kA(0.16);

    public static final GravityTypeValue kElevatorGravityType = GravityTypeValue.Elevator_Static;

    private static final Slot0Configs kElevatorPIDConfig = new Slot0Configs()
        .withKP(30)
        .withKD(1)
        .withKV(0.775)
        .withKA(0)
        .withKG(0.8)
        .withGravityType(kElevatorGravityType);

    public static final CurrentLimitsConfigs kElevatorCurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(70)
        .withStatorCurrentLimit(200);

    public static final FeedbackConfigs kElevatorFeedbackConfigs = new FeedbackConfigs()
        .withSensorToMechanismRatio(5.0);

    public static final SoftwareLimitSwitchConfigs kElevatorSoftLimit = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(9.5) 
        .withReverseSoftLimitThreshold(0.05);

    public static final TalonFXConfiguration kElevatorConfig = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive))
        .withMotionMagic(kElevatorMotionMagicConfig)
        .withSlot0(kElevatorPIDConfig)
        .withCurrentLimits(kElevatorCurrentLimits)
        .withFeedback(kElevatorFeedbackConfigs)
        .withSoftwareLimitSwitch(kElevatorSoftLimit);

    public static final double kElevatorTolerance = 0.05;

    public static final double kElevatorL1 = 3.1;
    public static final double kElevatorL2 = 1.3;
    public static final double kElevatorL3 = 3.6;
    public static final double kElevatorL4 = 6.7;
    public static final double kElevatorLowAlgae = 3.5;
    public static final double kElevatorHighAlgae = 5.7;
    public static final double kElevatorCradlePos = 2.3;
    public static final double kElevatorProcessor = 0; 
    public static final double kElevatorNet = 9.1;
    public static final double kElevatorAlgaeGround = 2.0;
    public static final double kElevatorClimb = 0.05;

    public static final double kElevatorOverCradle = 3;
    public static final double kElevatorOverHighAlgae = 4.5;
    public static final double kElevatorStow = 1.6; 
    public static final double kElevatorCloseToNet = 8.8;
}