package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class EndEffectorConstants {
    public static final int kEndEffectorRollerMotorID = 20;

    public static final double kProcessorOuttakePower = 1.0;
    public static final double kNetOuttakePower = 1.0;

    public static final double kCoralOuttakeToTroughPower = 0.5;
    public static final double kCoralOuttakeToL2Power = 0.3;
    public static final double kCoralOuttakeToL3Power = 0.3;
    public static final double kCoralOuttakeToL4Power = 0.75;

    public static final double kCoralOuttakeWaitToTrough = 0.5;        
    public static final double kCoralOuttakeWaitToL2 = 0.1;
    public static final double kCoralOuttakeWaitToL3 = 0.1;
    public static final double kCoralOuttakeWaitBeforeL4 = 0.05;
    public static final double kCoralOuttakeWaitToL4 = 0.1;
    public static final double kAlgaeOuttakeWait = 0.5;

    public static final double kHoldAlgaeCurrent = -60;
    public static final double kHoldCoralCurrent = -15;

    public static final int kHasCoralCurrent = -14;
    public static final int kHasAlgaeCurrent = -57;

    public static final CurrentLimitsConfigs kEndEffectorCurrentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(120)
        .withSupplyCurrentLimit(40);

    public static final MotorOutputConfigs kEndEffectorOutputConfigs = new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake); 

    public static final TalonFXConfiguration kEndEffectorRollerMotorConfig = new TalonFXConfiguration()
        .withCurrentLimits(kEndEffectorCurrentLimits)
        .withMotorOutput(kEndEffectorOutputConfigs);
}