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
    private boolean readyForElevator = false;

    public Compositions(Arm arm, Elevator elevator, EndEffector endEffector, Indexer indexer, Intake intake, KrakenSwerve swerve, ButtonBoardHandler buttonBoardHandler) {
        this.arm = arm;
        this.elevator = elevator;
        this.endEffector = endEffector;
        this.indexer = indexer;
        this.intake = intake;
        this.swerve = swerve;

        this.buttonBoardHandler = buttonBoardHandler;

        prepareElevatorForScoring.onTrue(
            Commands.runOnce(() -> readyForElevator = true)  
        );
    }

    public Command noPieceSequence() {
        /* Algae mode must be scheduled seperately from coral to avoid requirement conflicts */
        return Commands.either(Commands.none()/*new ScheduleCommand(getAlgaeSequence())*/, getCoralSequence(), buttonBoardHandler::inAlgaeMode);
    }

    private Command getCoralSequence() {
        return Commands.sequence(
            new ScheduleCommand(backgroundCoralMovement()),
            buttonBoardHandler.followSourcePath(),
            new AimAtCoral(swerve)
                .alongWith(Commands.waitUntil(endEffector::hasCoral)),
            scoreCoralSequence()
        );
    }

    public Command resumeCoralSequence() {
        return Commands.sequence(
            new ScheduleCommand(backgroundScoreSequence()),
            scoreCoralSequence()  
        );
    }

    private Command scoreCoralSequence() {
        return Commands.sequence(
            buttonBoardHandler.followReefPath(),
            Commands.runOnce(() -> readyForScoring = true),
            Commands.waitUntil(() -> !readyForScoring)
        );
    }

    public Command autoIntake() {
        return prepareForIntaking()
            .andThen(
                new AimAtCoral(swerve)
                .alongWith(new ScheduleCommand(backgroundCoralMovement()))
            );
    }

    private Command backgroundCoralMovement() {
        return Commands.sequence(
            /* Intake sequence */
            prepareForIntaking(),
            intake.intakeCoral(),
            indexer.spinRollers(),
            endEffector.receiveCoralFromIndexer(),
            intake.stopIntaking(),
            indexer.stopRollers(),
            intake.returnToStowPosition(),
            Commands.parallel(
                endEffector.holdCoral(),
                arm.moveToStowPos()
            ).until(() -> readyForElevator),
            /* Prepare and score when ready */
            backgroundScoreSequence()
        )
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public Command backgroundScoreSequence() {
        return Commands.sequence(
            Commands.waitUntil(() -> readyForElevator),
            prepareForCoralScoring(),
            Commands.waitUntil(() -> readyForScoring),//TODO how to do this better... need to wait until path is finished
            endEffector.placeCoral(),
            arm.moveToStowPos(),
            buttonBoardHandler.clearReefTargets(),
            Commands.runOnce(() -> { readyForScoring = false; readyForElevator = false; })
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
        return Commands.sequence(
            intake.moveToGround(),
            elevator.moveToIndexerPosition()
                .andThen(Commands.waitUntil(elevator::atSetpoint)),
            arm.moveToIndexerPos()
                .andThen(Commands.waitUntil(arm::atSetpoint))
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