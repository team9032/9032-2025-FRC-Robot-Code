package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class IntakeConstants {
    public static final CurrentLimitsConfigs kIntakeArmMotorCurrentLimit = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(40)
        .withStatorCurrentLimit(120);

    public static final GravityTypeValue kIntakeArmGravityType = GravityTypeValue.Arm_Cosine;

    public static final Slot0Configs kIntakeArmPidConstants = new Slot0Configs()
        .withKP(0.5)
        .withKV(0.023)
        .withGravityType(kIntakeArmGravityType);

    public static final SoftwareLimitSwitchConfigs kIntakeArmSoftLimit = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(0.0)
        .withReverseSoftLimitThreshold(-120.0); 

    public static final MotionMagicConfigs kIntakeArmMotionMagicConfigs = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(300)
        .withMotionMagicAcceleration(1000); 

    public static final FeedbackConfigs kIntakeArmFeedbackConfigs = new FeedbackConfigs()
        .withSensorToMechanismRatio(50.0 / 360.0); 

    public static final TalonFXConfiguration kIntakeArmConfig = new TalonFXConfiguration()
        .withCurrentLimits(kIntakeArmMotorCurrentLimit)
        .withSlot0(kIntakeArmPidConstants)
        .withMotionMagic(kIntakeArmMotionMagicConfigs)
        .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
        .withFeedback(kIntakeArmFeedbackConfigs)
        .withSoftwareLimitSwitch(kIntakeArmSoftLimit);

    public static final CurrentLimitsConfigs kIntakeRollerCurrentLimit = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(40)
        .withStatorCurrentLimit(120); 

    public static final TalonFXConfiguration kIntakeRollerConfig = new TalonFXConfiguration()
        .withCurrentLimits(kIntakeRollerCurrentLimit)
        .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
    
    public static final int kIntakeArmID = 16;
    public static final int kIntakeRollerID = 17;

    public static final double kEjectPower = 0.25;
    public static final double kEjectDelay = 0.75;

    public static final double kIntakePower = -1.0;

    public static final double kGroundPosition = -130.0;
    public static final double kStowPosition = -27.0;
    public static final double kEndEffectorMovePosition = -94.0;

    public static final double kRunRollersPosition = -104;

    public static final double kCanClimbPosition = -34.0;
}
