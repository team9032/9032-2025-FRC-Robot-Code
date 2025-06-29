package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ElasticUtil;

import static frc.robot.Constants.TransferConstants.*;

public class Transfer extends SubsystemBase {
    private final TalonFX rollerMotor;

    private final DigitalInput coralSensor = new DigitalInput(kTransferPhotoelectricSensorID); 

    public Transfer() {
        rollerMotor = new TalonFX(kTransferRollerID);
        ElasticUtil.checkStatus(rollerMotor.getConfigurator().apply(kTransferRollerConfig));
        
        rollerMotor.optimizeBusUtilization();
    }

    private Command spinRollers() {
        return runOnce(() -> rollerMotor.set(kTransferRollerPower));
    }

    public Command stopRollers() {
        return runOnce(() -> rollerMotor.set(0));
    }

    public Command receiveCoralFromIntake() {
        return Commands.sequence(
            spinRollers(),
            Commands.waitUntil(this::hasCoral),
            stopRollers()
        );
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
        SmartDashboard.putBoolean("Transfer has coral", hasCoral());
    }
}