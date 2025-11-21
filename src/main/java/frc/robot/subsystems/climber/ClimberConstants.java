package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberConstants {
    public static final int kClimberArmID = 32;
    public static final int kClimberIntakeID = 33;

    public static final double kClimberStowPos = 0.11;
    public static final double kClimberCageIntakePos = 0.18;
    public static final double kClimberClimbPos = -0.09;

    private static final Slot0Configs kClimberPIDConfig = new Slot0Configs()
        .withKP(10000)
        .withKD(0)
        .withKV(0)
        .withKA(0)
        .withKS(0)
        .withKG(0);
    //FIXME hi HARSHIL PANDENATOR

    public static final CurrentLimitsConfigs kClimberArmCurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(70)
        .withStatorCurrentLimit(120);
        
    public static final FeedbackConfigs kClimberArmFeedbackConfigs = new FeedbackConfigs()
        .withSensorToMechanismRatio(432.0); 

    public static final SoftwareLimitSwitchConfigs kClimberSoftLimit = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(0.25)
        .withReverseSoftLimitThreshold(-0.12);

    public static final TalonFXConfiguration kClimberArmMotorConfig = new TalonFXConfiguration()
        .withSlot0(kClimberPIDConfig)
        .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
        .withFeedback(kClimberArmFeedbackConfigs)
        .withCurrentLimits(kClimberArmCurrentLimits)
        .withSoftwareLimitSwitch(kClimberSoftLimit);

    public static final TalonFXConfiguration kClimberIntakeMotorConfig = new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(120)
            .withSupplyCurrentLimit(40)
        );

    public static final double kClimberIntakeVolts = 6.0;
    public static final int kHasCageCurrent = 94;
    public static final double kCageIntakeDelay = 0.5;

    public static final double kClimberArmTolerance = 0.005;
}
