package frc.robot.subsystems.transfer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class TransferConstants {
    public static final int kTransferRollerID = 15;
    public static final int kTransferPhotoelectricSensorID = 0;

    public static final double kTransferRollerPower = -1.0;
    public static final double kTransferRollerSlowPower = -0.5;

    public static final double kTransferFromSensorWait = 0.25;

    public static final double kTransferEjectPower = 0.25;
    public static final double kTransferEjectWait = 1.0;

    public static final CurrentLimitsConfigs kTransferRollerCurrentLimitConfigs = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(40)
        .withStatorCurrentLimit(120);

    public static final TalonFXConfiguration kTransferRollerConfig = new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
        .withCurrentLimits(kTransferRollerCurrentLimitConfigs);
}