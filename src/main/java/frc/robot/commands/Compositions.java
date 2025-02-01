package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.*;

/** Contains all command compositions that use multiple subsystems. Do not put single subsystem commands here. */
public class Compositions {
    private Compositions() {}

    public static Command intakeAndStoreCoral(Intake intake, Indexer indexer, Elevator elevator, EndEffector endEffector, Arm arm) { 
        return Commands.sequence(
            elevator.moveToIndexerPosition()//TODO Should be done earlier
                .alongWith(arm.moveToIndexerPos()),
            intake.intakeCoral(),
            indexer.spinRollersUntilCoralReceived(),
            intake.stopIntaking(),
            indexer.spinRollers(),//TODO make sure the elevator and arm are ready otherwise coral is launched into the robot
            endEffector.receiveCoralFromIndexer(),
            indexer.stopRollers()
        );
    }
}