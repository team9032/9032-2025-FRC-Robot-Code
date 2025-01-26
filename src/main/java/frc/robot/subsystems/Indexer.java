package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IndexerConstants.*;

public class Indexer extends SubsystemBase {
    private final TalonFX rollerMotor;
    private final DigitalInput sensor;

    public Indexer() {
        rollerMotor = new TalonFX(kRollerMotorID);
        sensor = new DigitalInput(kSensorPortID);
    }

    public Command spinRollers() {
        return runOnce(() -> rollerMotor.set(kRollerMotorSpeed));
    }

    public Command stopRollers() {
        return runOnce(() -> rollerMotor.set(0));
    }

    public boolean hasCoral() {
        return sensor.get();
    }

    public Command spinRollersUntilCoralReceived() {
        return spinRollers()
            .until(() -> hasCoral())
            .andThen(stopRollers());
    }
}