package frc.robot.automation;

import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.commands.AimAtCoral;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerve.KrakenSwerve;
import frc.robot.utils.ElasticUtil;

/** Contains all command compositions that use multiple subsystems. Do not put single subsystem commands here. */
public class Compositions {
    private final EndEffector endEffector;
    private final Indexer indexer;
    private final Intake intake;
    private final KrakenSwerve swerve;

    private final ButtonBoardHandler buttonBoardHandler;
    private final ElevatorArmIntakeHandler elevatorArmIntakeHandler;

    public final EventTrigger prepareElevatorForScoring = new EventTrigger("Elevator");

    public Compositions(ElevatorArmIntakeHandler elevatorArmIntakeHandler, EndEffector endEffector, Indexer indexer, Intake intake, KrakenSwerve swerve, ButtonBoardHandler buttonBoardHandler) {
        this.endEffector = endEffector;
        this.indexer = indexer;
        this.intake = intake;
        this.swerve = swerve;

        this.buttonBoardHandler = buttonBoardHandler;
        this.elevatorArmIntakeHandler = elevatorArmIntakeHandler;

        prepareElevatorForScoring.onTrue(
            Commands.waitSeconds(0.25)
                .onlyIf(buttonBoardHandler::l4Selected)
            .andThen(elevatorArmIntakeHandler.prepareForCoralScoring())
        );
    }

    public Command driveToSource() {
        return Commands.sequence(
            ElasticUtil.sendInfoCommand("Drive to source started"),
            new ScheduleCommand(elevatorArmIntakeHandler.moveToIntakePosition()),
            Commands.waitSeconds(0.25),//TODO no
            buttonBoardHandler.followSourcePath()
        );
    }

    public Command alignToReefAndScore() {
        return Commands.sequence(
            ElasticUtil.sendInfoCommand("Aligning to reef and scoing"),
            buttonBoardHandler.followReefPath(swerve),//This will trigger the elevator and arm
            Commands.waitUntil(elevatorArmIntakeHandler::readyForCoralScoring),
            Commands.waitSeconds(0.25)//TODO fix?
                .onlyIf(buttonBoardHandler::l4Selected),
            buttonBoardHandler.scoreCoral(endEffector).asProxy()
        );
    }

    public Command autoIntake(boolean moveToStow) {
        return Commands.sequence(
            ElasticUtil.sendInfoCommand("Started auto intaking"),
            intake.resetLastObstacleDistance(),
            new AimAtCoral(swerve, intake::getObstacleSensorDistance, true)
                .until(endEffector::hasCoral)
                    .alongWith(intakeCoralToEndEffector(moveToStow))  
        );
    }

    public Command pulseIntake() {
        return Commands.sequence(
            ElasticUtil.sendInfoCommand("Pulsing intake"),
            elevatorArmIntakeHandler.moveToIntakePosition(),
            intake.intakeCoral(),
            indexer.spinRollers(),
            endEffector.receiveCoralFromIndexer().asProxy()
                .alongWith(
                    /* Pulse */
                    Commands.sequence(
                        intake.moveToEndEffectorMovePosition(),
                        Commands.waitSeconds(0.1),
                        intake.moveToGround()
                    )
                ),
            intake.stopIntaking(),
            indexer.stopRollers(),
            new ScheduleCommand(endEffector.holdCoral()),
            elevatorArmIntakeHandler.moveToStowPositions()
                .onlyIf(buttonBoardHandler::l1NotSelected)
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
            elevatorArmIntakeHandler.moveToStowPositions()
                .onlyIf(() -> moveToStow && buttonBoardHandler.l1NotSelected())
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

    // public Command intakeAlgaeFromReef() {
    //     return Commands.sequence(
    //         buttonBoardHandler.followAlgaeIntakePath(swerve)
    //             .alongWith(elevatorArmIntakeHandler.prepareForAlgaeIntaking()),
    //         endEffector.pickupAlgae()
    //     );
    // }

    public Command scoreAlgaeSequence() {
        return Commands.sequence(
            buttonBoardHandler.followAlgaeScorePath()
                .alongWith(elevatorArmIntakeHandler.prepareForAlgaeScoring()),//TODO handle net algae
            buttonBoardHandler.scoreAlgae(endEffector)
        );
    }

    public Command stopRollers() {
        return Commands.sequence(
            intake.stopIntaking(),
            indexer.stopRollers(),
            endEffector.stopRollers()
        );
    }
}