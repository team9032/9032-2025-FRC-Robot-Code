package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConfigs;

public class Elevator extends SubsystemBase {
    private final TalonFX elevatorMotor;

    public Elevator() {
        elevatorMotor = new TalonFX(25); // TODO change CAN stuff
        elevatorMotor.getConfigurator().apply(ElevatorConfigs.kELevatorMotorConfig);

    }

    public void moveElevator(double pos) {
        elevatorMotor.setControl(new PositionVoltage(pos));
    }

    public Command elevatorCommand(double pos) {
        return runOnce(() -> moveElevator(pos));
    }
}