package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.EndEffectorConstants.*;

public class EndEffector extends SubsystemBase {
    TalonFX endEffectorMotor = new TalonFX(kEndEffectorID);

    DigitalInput photoelectricSensor = new DigitalInput(kPhotoelectricSensorID);

    public EndEffector() {
        endEffectorMotor.getConfigurator().apply(kEndEffectorConfig);
    }

    private Command setEndEffectorMotor(double power) {
        return runOnce(() -> endEffectorMotor.set(kOuttakePower));
    }

    public Command placeCoral() {
        return Commands.sequence(
            setEndEffectorMotor(kOuttakePower),
            Commands.waitSeconds(kOuttakeWait),
            setEndEffectorMotor(0.0)
        );
    }

    public Command pickupCoralFromSource() {
        return Commands.sequence(
            setEndEffectorMotor(kIntakeFromSourcePower),
            Commands.waitUntil(() -> photoelectricSensor.get()),//TODO this sensor will not be placed in the right spot - need to wait 
            setEndEffectorMotor(0.0)
        );
    }

    public Command receiveCoralFromIndexer() {
        return Commands.sequence(
            setEndEffectorMotor(kReceiveFromIndexerPower),
            Commands.waitUntil(() -> photoelectricSensor.get()),
            setEndEffectorMotor(0.0)
        );
    }
    //TODO add algae functionality
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("End Effector Photoelectric Sensor", photoelectricSensor.get());
    }
}