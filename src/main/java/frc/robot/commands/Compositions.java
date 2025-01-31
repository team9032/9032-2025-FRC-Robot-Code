package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.*;

/** Contains all command compositions that use multiple subsystems. Do not put single subsystem commands here. */
public class Compositions {
    private Compositions() {}

    public static Command autoIntakeCoralSequence(Indexer indexer, Elevator elevator, EndEffector endEffector) { 
        return Commands.sequence(
            indexer.spinRollersUntilCoralReceived(),
            elevator.elevatorL1Command(),
            endEffector.pickupCoral()
        );
    }
}