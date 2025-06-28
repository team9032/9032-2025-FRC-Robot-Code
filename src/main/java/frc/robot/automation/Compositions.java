package frc.robot.automation;

import java.util.function.Supplier;

import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.automation.ButtonBoardHandler.ReefLevel;
import frc.robot.automation.ButtonBoardHandler.ReefPath;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerve.KrakenSwerve;
import frc.robot.utils.ElasticUtil;

/** Contains all command compositions that use multiple subsystems. Do not put single subsystem commands here. */
public class Compositions {
    private final EndEffector endEffector;
    private final Indexer indexer;
    private final Intake intake;
    private final Climber climber;
    private final KrakenSwerve swerve;

    private final ButtonBoardHandler buttonBoardHandler;
    private final ElevatorArmIntakeHandler elevatorArmIntakeHandler;

    private final EventTrigger prepareElevatorForCoralScoring = new EventTrigger("Elevator");
    private final EventTrigger prepareElevatorForAlgaeScoring = new EventTrigger("ElevatorAlgae");

    public Compositions(ElevatorArmIntakeHandler elevatorArmIntakeHandler, EndEffector endEffector, Indexer indexer, Intake intake, Climber climber, KrakenSwerve swerve, ButtonBoardHandler buttonBoardHandler) {
        this.endEffector = endEffector;
        this.indexer = indexer;
        this.intake = intake;
        this.climber = climber;
        this.swerve = swerve;

        this.buttonBoardHandler = buttonBoardHandler;
        this.elevatorArmIntakeHandler = elevatorArmIntakeHandler;
    }

    public Command driveToSource() {
        return Commands.sequence(
            ElasticUtil.sendInfoCommand("Drive to source started"),
            new ScheduleCommand(elevatorArmIntakeHandler.moveToIntakePosition()),
            Commands.waitSeconds(0.25),//TODO no
            PathfindingHandler.pathToSource(buttonBoardHandler::getSelectedSourcePath)
        );
    }

    public Command alignToReefAndScoreFromButtonBoard() {
        return alignToReefAndScore(buttonBoardHandler::getSelectedReefPath, buttonBoardHandler::getSelectedReefLevel);
    }

    public Command alignToReefAndScoreFromPreset(ReefPath reefPath, ReefLevel reefLevel) {
        return alignToReefAndScore(() -> reefPath, () -> reefLevel);
    }

    public Command alignToReefAndScore(Supplier<ReefPath> reefPathSup, Supplier<ReefLevel> reefLevelSup) {
        return Commands.sequence(
            ElasticUtil.sendInfoCommand("Aligning to reef and scoring in auto"),
            PathfindingHandler.pathToReefSide(reefPathSup)
                /* Moves elevator and arm when the robot hits the event trigger */
                .alongWith(
                    Commands.waitUntil(prepareElevatorForCoralScoring)
                    .andThen(
                        Commands.waitSeconds(0.25)
                            .onlyIf(() -> reefLevelSup.get().equals(ReefLevel.L4)),
                        elevatorArmIntakeHandler.prepareForCoralScoring(reefLevelSup)
                    )   
                ),
            Commands.waitUntil(() -> elevatorArmIntakeHandler.readyToScoreCoral(reefLevelSup.get())),
            Commands.waitSeconds(0.25)//TODO fix?
                .onlyIf(() -> reefLevelSup.get().equals(ReefLevel.L4)),
            endEffector.scoreCoral(buttonBoardHandler::getSelectedReefLevel).asProxy()
        );
    }

    public Command intakeNearestCoral(boolean moveToStow) {
        return Commands.sequence(
            ElasticUtil.sendInfoCommand("Started intaking nearest coral"),
            PathfindingHandler.pathToNearestCoral(swerve)
                .alongWith(intakeCoralToEndEffector(moveToStow))
        );
    }

    public Command pulseIntake() {
        return Commands.sequence(
            ElasticUtil.sendInfoCommand("Pulsing intake"),
            cancelIntake(),
            Commands.waitSeconds(0.08),
            intakeCoralToEndEffector(true)
        )
        .onlyIf(() -> !endEffector.hasCoral() && !endEffector.hasAlgae());
    }

    public Command intakeCoralToEndEffector(boolean moveToStow) {
        return Commands.sequence(
            ElasticUtil.sendInfoCommand("Started intaking"),
            elevatorArmIntakeHandler.moveToIntakePosition(),
            intake.intakeCoral(),
            indexer.spinRollers(),
            endEffector.receiveCoralFromIndexer().asProxy(),
            intake.stopIntaking(),
            indexer.stopRollers(),
            new ScheduleCommand(endEffector.holdCoral()),
            /* Prepare for L1 early instead of stowing */
            Commands.either(
                elevatorArmIntakeHandler.prepareForCoralScoring(() -> ReefLevel.L1), 
                elevatorArmIntakeHandler.moveToStowPositions()
                    .onlyIf(() -> moveToStow), 
                () -> buttonBoardHandler.getSelectedReefLevel().equals(ReefLevel.L1)
            )
        )
        .onlyIf(() -> !endEffector.hasCoral() && !endEffector.hasAlgae());
    }

    public Command cancelIntake() {
        return Commands.sequence(
            ElasticUtil.sendInfoCommand("Canceled intaking"),
            elevatorArmIntakeHandler.moveIntakeUp(),
            intake.outtakeCoral(),
            /* Recover from coral partially in the end effector */
            Commands.either(
                endEffector.receiveCoralFromIndexer().asProxy()
                    .andThen(
                        ElasticUtil.sendInfoCommand("Recovering from coral partially in end effector"),
                        new ScheduleCommand(endEffector.holdCoral()),
                        elevatorArmIntakeHandler.moveToStowPositions()
                    ),
                endEffector.stopRollers().asProxy(),
                () -> !endEffector.hasCoralCentered() && endEffector.hasCoral()
            ),
            indexer.stopRollers(),
            intake.stopIntaking()
        );
    }       

    public Command intakeAlgaeFromReef() {
        return Commands.sequence(
            Commands.print("Intaking algae from reef"),
            PathfindingHandler.pathToAlgaeIntakeFromReef(buttonBoardHandler::getSelectedReefPath)
            .alongWith(
                /* Low algae position can be reached while driving, but high can't because of tipping */
                Commands.either(
                    elevatorArmIntakeHandler.prepareForAlgaeReefIntaking(buttonBoardHandler::lowAlgaeSelected), 
                    /* Moves elevator and arm to high algae intake when the robot hits the event trigger  */
                    Commands.waitUntil(prepareElevatorForAlgaeScoring)
                        .deadlineFor(elevatorArmIntakeHandler.moveToStowPositions())
                        .andThen(elevatorArmIntakeHandler.prepareForAlgaeReefIntaking(buttonBoardHandler::lowAlgaeSelected)), 
                    buttonBoardHandler::lowAlgaeSelected
                ),
                endEffector.pickupAlgae()
            )
        );
    }

    public Command scoreAlgaeSequence() {
        return Commands.sequence(
            PathfindingHandler.followAlgaeScorePath(buttonBoardHandler::getSelectedAlgaeScorePath),
            elevatorArmIntakeHandler.prepareForAlgaeScoring(buttonBoardHandler::getSelectedAlgaeScorePath),
            endEffector.scoreAlgae(buttonBoardHandler::getSelectedAlgaeScorePath)
        );
    }

    public Command climb() {
        return elevatorArmIntakeHandler.prepareForClimbing()
            .andThen(climber.intakeCageAndClimb());
    }

    public Command cancelClimb() {
        return elevatorArmIntakeHandler.moveToStowPositions()
            .andThen(climber.moveToStowPosition());
    }

    public Command stopRollers() {
        return Commands.sequence(
            intake.stopIntaking(),
            indexer.stopRollers(),
            endEffector.stopRollers()
        );
    }
}