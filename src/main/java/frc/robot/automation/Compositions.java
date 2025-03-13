package frc.robot.automation;

import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.commands.AimAtCoral;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerve.KrakenSwerve;
import frc.robot.utils.ElasticUtil;

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
    private final EventTrigger readyForIntaking = new EventTrigger("Intake");

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
        return Commands.either(Commands.none()/*new ScheduleCommand(getAlgaeSequence())*/, getCoralSequence(true, true), buttonBoardHandler::inAlgaeMode);
    }

    public Command getCoralSequence(boolean goToSource, boolean continueToScoring) {
        return Commands.sequence(
            ElasticUtil.sendInfoCommand("Get coral sequence started - go to source is " + goToSource),
            new ScheduleCommand(backgroundCoralMovement(goToSource)),
            buttonBoardHandler.followSourcePath()
                .onlyIf(() -> goToSource),
            Commands.waitUntil(intake::canRunRollers),
            intake.resetLastObstacleDistance(),//Does not require intake subsystem
            new AimAtCoral(swerve, intake::getObstacleSensorDistance)
                .alongWith(Commands.waitUntil(endEffector::hasCoral)),
            ElasticUtil.sendInfoCommand("Got coral - starting score coral sequence is " + continueToScoring),
            scoreCoralSequence()
                .onlyIf(() -> continueToScoring)
        );
    }

    public Command resumeCoralSequence() {
        return Commands.sequence(
            ElasticUtil.sendInfoCommand("Resume coral sequence started - starting score coral sequence"),
            new ScheduleCommand(backgroundScoreSequence()),
            scoreCoralSequence()  
        );
    }

    private Command scoreCoralSequence() {
        return Commands.sequence(
            Commands.waitUntil(buttonBoardHandler::hasQueues),
            buttonBoardHandler.followReefPath(),
            Commands.runOnce(() -> readyForScoring = true),
            Commands.waitUntil(() -> !readyForScoring)
        );
    }

    private Command backgroundCoralMovement(boolean goingToSource) {
        return Commands.sequence(
            /* Intake sequence */
            ElasticUtil.sendInfoCommand("Background coral movement started - going to source " + goingToSource),
            prepareForIntaking(),
            Commands.waitUntil(readyForIntaking)
                .onlyIf(() -> goingToSource),
            intake.moveToGround(),
            Commands.waitUntil(intake::canRunRollers),
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
        );
    }

    public Command backgroundScoreSequence() {
        return Commands.sequence(
            ElasticUtil.sendInfoCommand("Background score sequence started"),
            endEffector.holdCoral() 
                .until(() -> readyForElevator),
            prepareForCoralScoring()
                .alongWith(
                    endEffector.holdCoral()
                        .until(() -> readyForScoring)
                ),
            Commands.waitUntil(() -> readyForScoring),
            endEffector.placeCoral(),
            arm.moveToStowPos(),
            buttonBoardHandler.clearReefTargets(),
            Commands.runOnce(() -> { readyForScoring = false; readyForElevator = false; })
        );
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
            elevator.moveToIndexerPosition(),
            Commands.waitUntil(elevator::atSetpoint),
            arm.moveToIndexerPos(),
            Commands.waitUntil(arm::atSetpoint),
            ElasticUtil.sendInfoCommand("Prepared for intaking")
        );
    }

    private Command prepareForCoralScoring() {
        return Commands.sequence(
            buttonBoardHandler.moveElevatorToCoralTargetLevel(elevator),
            Commands.waitUntil(elevator::atSetpoint),
            buttonBoardHandler.moveArmToCoralTargetLevel(arm),
            Commands.waitUntil(arm::atSetpoint),
            ElasticUtil.sendInfoCommand("Prepared for coral scoring")
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

    public Command resetStates() {
        return Commands.sequence(
            intake.stopIntaking(),
            indexer.stopRollers(),
            endEffector.stopRollers(),
            Commands.runOnce(() -> { 
                readyForScoring = false; 
                readyForElevator = false; 
                
                if (arm.getCurrentCommand() != null)
                    arm.getCurrentCommand().cancel();

                if (elevator.getCurrentCommand() != null)
                    elevator.getCurrentCommand().cancel();
            })
        );
    }
}