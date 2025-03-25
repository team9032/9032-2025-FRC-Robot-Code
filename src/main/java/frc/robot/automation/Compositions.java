package frc.robot.automation;

import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AimAtCoral;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerve.KrakenSwerve;
import frc.robot.utils.ElasticUtil;
import frc.robot.utils.LocalizationTrigger;

import static frc.robot.Constants.AutomationConstants.kIntakeZoneRectangle;

/** Contains all command compositions that use multiple subsystems. Do not put single subsystem commands here. */
public class Compositions {
    private final EndEffector endEffector;
    private final Indexer indexer;
    private final Intake intake;
    private final KrakenSwerve swerve;

    private final ButtonBoardHandler buttonBoardHandler;
    private final ElevatorArmIntakeHandler elevatorArmIntakeHandler;

    public final EventTrigger prepareElevatorForScoring = new EventTrigger("Elevator");
    private final EventTrigger sourcePathHit = new EventTrigger("Intake");

    private final Trigger intakeDown;

    private boolean finishedReefPath = false;
    private boolean readyForElevator = false;
    private boolean readyForIntaking = false;

    public Compositions(ElevatorArmIntakeHandler elevatorArmIntakeHandler, EndEffector endEffector, Indexer indexer, Intake intake, KrakenSwerve swerve, ButtonBoardHandler buttonBoardHandler) {
        this.endEffector = endEffector;
        this.indexer = indexer;
        this.intake = intake;
        this.swerve = swerve;

        this.buttonBoardHandler = buttonBoardHandler;
        this.elevatorArmIntakeHandler = elevatorArmIntakeHandler;

        intakeDown = new LocalizationTrigger(swerve, kIntakeZoneRectangle).getTrigger();

        prepareElevatorForScoring.onTrue(
            Commands.runOnce(() -> readyForElevator = true)  
        );

        sourcePathHit.onTrue(
            Commands.runOnce(() -> readyForIntaking = true)
        );
    }

    public Command noCoralSequence() {
        return getCoralSequence(true, true);
    }

    public Command getCoralSequence(boolean goToSource, boolean continueToScoring) {
        return Commands.sequence(
            ElasticUtil.sendInfoCommand("Get coral sequence started - go to source is " + goToSource),
            new ScheduleCommand(backgroundCoralMovement(goToSource)),
            buttonBoardHandler.followSourcePath()
                .onlyIf(() -> goToSource),
            Commands.waitUntil(intake::canRunRollers),
            intake.resetLastObstacleDistance(),//Does not require intake subsystem
            new AimAtCoral(swerve, intake::getObstacleSensorDistance, false)
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
            Commands.runOnce(() -> finishedReefPath = true),
            Commands.waitUntil(() -> !finishedReefPath)
        );
    }

    public Command intakeCoralToEndEffector() {
        return Commands.sequence(
            ElasticUtil.sendInfoCommand("Started intaking"),
            elevatorArmIntakeHandler.moveToIntakePosition(true),
            intake.intakeCoral(),
            indexer.spinRollers(),
            endEffector.receiveCoralFromIndexer().asProxy(),
            intake.stopIntaking(),
            indexer.stopRollers(),
            new ScheduleCommand(endEffector.holdCoral()),
            elevatorArmIntakeHandler.moveToStowPositions()
        )
        .onlyIf(() -> !endEffector.hasCoral() && !endEffector.hasAlgae());
    }

    public Command cancelIntake() {
        return Commands.sequence(
            ElasticUtil.sendInfoCommand("Canceled intaking"),
            elevatorArmIntakeHandler.moveIntakeUp(),
            intake.stopIntaking(),
            endEffector.stopRollers(),
            indexer.stopRollers()
        );
    }       

    public Command backgroundCoralMovement(boolean goingToSource) {
        return Commands.sequence(
            /* Intake sequence */
            ElasticUtil.sendInfoCommand("Background coral movement started - going to source " + goingToSource),
            elevatorArmIntakeHandler.moveToIntakePosition(false),
            Commands.waitUntil(() -> readyForIntaking)
                .onlyIf(() -> goingToSource),
            intake.moveToGround(),
            Commands.waitUntil(intake::canRunRollers),
            intake.intakeCoral(),
            indexer.spinRollers(),
            endEffector.receiveCoralFromIndexer().asProxy(),
            // new ScheduleCommand(endEffector.holdCoral()),
            intake.stopIntaking(),
            indexer.stopRollers(),
            elevatorArmIntakeHandler.moveToStowPositions(),
            Commands.waitUntil(() -> readyForElevator),
            /* Prepare and score when ready */
            backgroundScoreSequence()
        );
    }

    private Command backgroundScoreSequence() {
        return Commands.sequence(
            ElasticUtil.sendInfoCommand("Background score sequence started"),
            Commands.waitUntil(() -> readyForElevator),
            elevatorArmIntakeHandler.prepareForCoralScoring(),
            Commands.waitUntil(() -> finishedReefPath),
            Commands.waitSeconds(10000000),
            buttonBoardHandler.scoreCoral(endEffector).asProxy(),
            Commands.runOnce(() -> { finishedReefPath = false; readyForElevator = false; readyForIntaking = false; }),
            elevatorArmIntakeHandler.moveToIntakePosition(false)
        );
    }

    public Command getAlgaeSequence() {
        return Commands.sequence(
            buttonBoardHandler.followAlgaeIntakePath()
                .alongWith(elevatorArmIntakeHandler.prepareForAlgaeIntakingFinal()),
            endEffector.pickupAlgae(),
            scoreAlgaeSequence()
        );
    }

    public Command scoreAlgaeSequence() {
        return Commands.sequence(
            buttonBoardHandler.followAlgaeScorePath()
                .alongWith(elevatorArmIntakeHandler.prepareForAlgaeScoring()),//TODO handle net algae
            buttonBoardHandler.scoreAlgae(endEffector)
        );
    }

    public Command resetStates() {
        return Commands.sequence(
            intake.stopIntaking(),
            indexer.stopRollers(),
            endEffector.stopRollers(),
            Commands.runOnce(() -> { 
                finishedReefPath = false; 
                readyForElevator = false; 
                readyForIntaking = false;
            })
        );
    }
}