package frc.robot.automation;

import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.commands.AimAtCoral;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerve.KrakenSwerve;

/** Contains all command compositions that use multiple subsystems. Do not put single subsystem commands here. */
public class Compositions {
    private final Arm arm;
    private final Elevator elevator;
    private final EndEffector endEffector;
    private final Indexer indexer;
    private final Intake intake;
    private final KrakenSwerve swerve;

    private final ButtonBoardHandler buttonBoardHandler;

    private final EventTrigger prepareElevatorForScoring = new EventTrigger("Elevator");

    private boolean readyForScoring = false;

    public Compositions(Arm arm, Elevator elevator, EndEffector endEffector, Indexer indexer, Intake intake, KrakenSwerve swerve, ButtonBoardHandler buttonBoardHandler) {
        this.arm = arm;
        this.elevator = elevator;
        this.endEffector = endEffector;
        this.indexer = indexer;
        this.intake = intake;
        this.swerve = swerve;

        this.buttonBoardHandler = buttonBoardHandler;
    }

    public Command noPieceSequence() {
        /* Algae mode must be scheduled seperately from coral to avoid requirement conflicts */
        return Commands.either(new ScheduleCommand(getAlgaeSequence()), getCoralSequence(), buttonBoardHandler::inAlgaeMode);
    }

    private Command getCoralSequence() {
        return Commands.sequence(
            new ScheduleCommand(backgroundCoralMovement()),
            buttonBoardHandler.followSourcePath(),
            new AimAtCoral(swerve).until(intake::hasCoral),
            scoreCoralSequence()
        );
    }

    public Command scoreCoralSequence() {
        return Commands.sequence(
            buttonBoardHandler.followReefPath(),
            Commands.runOnce(() -> readyForScoring = true),
            buttonBoardHandler.clearReefTargets()
        );
    }

    private Command backgroundCoralMovement() {
        return Commands.sequence(
            prepareForIntaking(),
            /* Intake sequence */
            intake.intakeCoral(),
            indexer.spinRollers(),
            endEffector.receiveCoralFromIndexer(),
            intake.stopIntaking(),
            indexer.stopRollers(),
            intake.returnToStowPosition(),
            endEffector.holdCoral(),
            /* Prepare and score when ready */
            arm.moveToStowPos(),
            Commands.waitUntil(prepareElevatorForScoring::getAsBoolean),
            prepareForCoralScoring(),
            Commands.waitUntil(() -> readyForScoring),//TODO how to do this better... need to wait until path is finished
            endEffector.placeCoral(),
            Commands.runOnce(() -> readyForScoring = false)
        )
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    private Command getAlgaeSequence() {
        return Commands.sequence(
            buttonBoardHandler.followAlgaeIntakePath()
                .alongWith(prepareForAlgaeIntaking()),
            endEffector.pickupAlgae(),
            scoreAlgaeSequence()
        );
    }

    public Command scoreAlgaeSequence() {
        return Commands.sequence(
            buttonBoardHandler.followAlgaeScorePath()
                .alongWith(prepareForAlgaeScoring()),
            buttonBoardHandler.scoreAlgae(endEffector),
            buttonBoardHandler.clearAlgaeTargets()
        );
    }

    private Command prepareForIntaking() {
        return Commands.parallel(
            intake.moveToGround(),
            arm.moveToIndexerPos()
                .andThen(Commands.waitUntil(arm::atSetpoint)),
            elevator.moveToIndexerPosition()
                .andThen(Commands.waitUntil(elevator::atSetpoint))
        );
    }

    private Command prepareForCoralScoring() {
        return Commands.sequence(
            buttonBoardHandler.moveElevatorToCoralTargetLevel(elevator),
            Commands.waitUntil(elevator::atSetpoint),
            buttonBoardHandler.moveArmToCoralTargetLevel(arm),
            Commands.waitUntil(arm::atSetpoint)
        );
    }

    private Command prepareForAlgaeIntaking() {
        return Commands.parallel(
            buttonBoardHandler.moveArmToAlgaeIntakeTargetLevel(arm)
                .andThen(Commands.waitUntil(arm::atSetpoint)),
            buttonBoardHandler.moveElevatorToAlgaeIntakeTargetLevel(elevator)
                .andThen(Commands.waitUntil(elevator::atSetpoint))
        );
    }

    private Command prepareForAlgaeScoring() {
        return Commands.parallel(
            buttonBoardHandler.moveArmToAlgaeScoreLevel(arm)
                .andThen(Commands.waitUntil(arm::atSetpoint)),
            buttonBoardHandler.moveElevatorToAlgaeScoreLevel(elevator)
                .andThen(Commands.waitUntil(elevator::atSetpoint))
        );
    }
}