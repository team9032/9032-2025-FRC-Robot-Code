package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.EndEffectorConstants.*;

public class EndEffector extends SubsystemBase {
    TalonFX endEffectorMotor = new TalonFX(kEndEffectorID);

    DigitalInput endEffectorBeamBreak = new DigitalInput(kEndEffectorBeamBreakID);

    public EndEffector() {
        endEffectorMotor.getConfigurator().apply(kEndEffectorConfig);
    }

    public Command placeCoral() {
        return Commands.sequence(
            runOnce(() -> endEffectorMotor.set(kEndEffectorOuttakeSpeed)),
            Commands.waitSeconds(0.5),
            runOnce(() -> endEffectorMotor.set(0))
        );
    }

    public Command pickupCoral() {
        return Commands.sequence(
            runOnce(() -> endEffectorMotor.set(-kEndEffectorIntakeSpeed)),
            Commands.waitUntil(() -> endEffectorBeamBreak.get()),
            runOnce(() -> endEffectorMotor.set(0))
        );
    }
}