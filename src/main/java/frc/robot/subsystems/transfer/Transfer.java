package frc.robot.subsystems.transfer;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ElasticUtil;

import static frc.robot.Constants.DriverConstants.kCANBusName;
import static frc.robot.subsystems.transfer.TransferConstants.*;

public class Transfer extends SubsystemBase {
    private final TalonFX rollerMotor;

    private final DigitalInput coralSensor = new DigitalInput(kTransferPhotoelectricSensorID); 

    public Transfer() {
        rollerMotor = new TalonFX(kTransferRollerID, kCANBusName);
        ElasticUtil.checkStatus(rollerMotor.getConfigurator().apply(kTransferRollerConfig));        
    }

    private Command spinRollers() {
        return runOnce(() -> rollerMotor.set(kTransferRollerPower));
    }

    private Command spinRollersSlowly() {
        return runOnce(() -> rollerMotor.set(kTransferRollerSlowPower));
    }

    public Command stopRollers() {
        return runOnce(() -> rollerMotor.set(0));
    }

    public Command receiveCoralFromIntakeForPickup() {
        return Commands.sequence(
            spinRollers(),
            Commands.waitUntil(this::hasCoral),
            spinRollersSlowly()
        )
        .onlyIf(() -> !hasCoral());
    }

    public Command receiveCoralFromIntake() {
        return Commands.sequence(
            spinRollers(),
            Commands.waitUntil(this::hasCoral),
            spinRollersSlowly(),
            Commands.waitSeconds(kTransferFromSensorWait),
            stopRollers()
        )
        .onlyIf(() -> !hasCoral());
    }

    public Command eject() {
        return runOnce(() -> rollerMotor.set(kTransferEjectPower))
            .andThen(
                Commands.waitSeconds(kTransferEjectWait),
                stopRollers()
            );
    }

    public boolean hasCoral() {
        return !coralSensor.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Transfer Sensor", hasCoral());
    }
}