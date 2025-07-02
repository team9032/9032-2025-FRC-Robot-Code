package frc.robot.automation;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.automation.ButtonBoardHandler.ReefLevel;
import frc.robot.automation.ButtonBoardHandler.ReefPath;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerve.KrakenSwerve;
import frc.robot.utils.ElasticUtil;
import frc.robot.utils.FieldUtil;

/** Contains all command compositions that use multiple subsystems. Do not put single subsystem commands here. */
public class Compositions {
    private final EndEffector endEffector;
    private final Transfer transfer;
    private final Intake intake;
    private final Climber climber;
    private final KrakenSwerve swerve;

    private final ButtonBoardHandler buttonBoardHandler;
    private final ElevatorArmIntakeHandler elevatorArmIntakeHandler;

    public Compositions(ElevatorArmIntakeHandler elevatorArmIntakeHandler, EndEffector endEffector, Transfer transfer, Intake intake, Climber climber, KrakenSwerve swerve, ButtonBoardHandler buttonBoardHandler) {
        this.endEffector = endEffector;
        this.transfer = transfer;
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
            PathfindingHandler.pathToSource(buttonBoardHandler::getSelectedSourcePath)
        );
    }

    public Command alignToReefAndScoreFromPreset(ReefPath reefPath, ReefLevel reefLevel) {
        return Commands.none();//TODO
    }

    public Command alignToReefAndScoreInterruptable(boolean isLeftBranch, Supplier<ReefLevel> reefLevelSup, BooleanSupplier shouldInterrupt) {
        return Commands.sequence(
            ElasticUtil.sendInfoCommand("Aligning to reef and scoring"),
            PathfindingHandler.pathToClosestReefBranch(swerve, isLeftBranch)
                /* Moves the elevator and arm when the robot is close enough to the reef */
                .alongWith(
                    Commands.waitUntil(() -> FieldUtil.shouldPrepareToScoreCoral(swerve.getLocalization()))
                    .andThen(elevatorArmIntakeHandler.prepareForCoralScoring(reefLevelSup))   
                ),
            Commands.waitUntil(() -> elevatorArmIntakeHandler.readyToScoreCoral(reefLevelSup.get())),
            elevatorArmIntakeHandler.moveArmToCoralScorePos(reefLevelSup),
            endEffector.scoreCoral(reefLevelSup)
        )
        .until(shouldInterrupt)
        .andThen(
            Commands.either(
                elevatorArmIntakeHandler.moveToStowPositions(),
                Commands.sequence(
                    Commands.waitUntil(() -> FieldUtil.endEffectorCanClearReef(swerve.getLocalization())),
                    elevatorArmIntakeHandler.moveToIntakePosition()   
                ),
                endEffector::hasCoral
            )
        )
        .onlyIf(endEffector::hasCoral);
    }

    public Command alignToReefAndScore(boolean isLeftBranch, Supplier<ReefLevel> reefLevelSup) {
        return alignToReefAndScoreInterruptable(isLeftBranch, reefLevelSup, () -> false);
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
            elevatorArmIntakeHandler.moveToIntakePosition()
                .alongWith(
                    Commands.sequence(
                        intake.moveToGround(),
                        Commands.waitUntil(intake::canRunRollers),
                        intake.intakeCoral(),
                        transfer.receiveCoralFromIntake()
                    )
                ),
            intake.stopIntaking(),
            elevatorArmIntakeHandler.moveToCoralCradlePosition(),
            endEffector.pickupCoralFromCradle(),
            Commands.waitUntil(() -> FieldUtil.endEffectorCanClearReef(swerve.getLocalization())),//Don't hit the reef when moving to stow
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
            intake.ejectCoral()
                .alongWith(transfer.eject())
        );
    }       

    public Command intakeNearestAlgaeFromReef() {
        return Commands.sequence(
            Commands.print("Intaking algae from the reef"),
            PathfindingHandler.pathToClosestReefAlgaeIntake(swerve)
            .alongWith(
                Commands.sequence(
                    Commands.waitUntil(() -> FieldUtil.shouldPrepareToIntakeAlgae(swerve.getLocalization())),
                    elevatorArmIntakeHandler.prepareForAlgaeReefIntaking(() -> FieldUtil.isClosestReefLocationHighAlgae(swerve.getLocalization())),
                    endEffector.intakeAlgae()
                )
            )
        )
        .onlyIf(() -> !endEffector.hasCoral());
    }

    public Command scoreAlgaeInNet() {
        return Commands.sequence(
            PathfindingHandler.pathToBarge(swerve)
                .alongWith(
                    Commands.sequence(
                        elevatorArmIntakeHandler.moveToStowPositions(),
                        Commands.waitUntil(() -> FieldUtil.shouldPrepareToScoreNetAlgae(swerve.getLocalization())),
                        elevatorArmIntakeHandler.prepareForNetAlgaeScoring()
                    )
                ),
            endEffector.scoreAlgae(buttonBoardHandler::getSelectedAlgaeScorePath),
            elevatorArmIntakeHandler.moveToStowPositionsFromNet()
        )
        .onlyIf(endEffector::hasAlgae);
    }

    public Command intakeGroundAlgae() {
        return Commands.sequence(
            elevatorArmIntakeHandler.prepareForAlgaeGroundIntaking(),
            endEffector.intakeAlgae()  
        )
        .onlyIf(() -> !endEffector.hasAlgae() && !endEffector.hasCoral());
    }

    public Command climb() {
        return elevatorArmIntakeHandler.prepareForClimbing()
            .andThen(climber.intakeCageAndClimb());
    }

    public Command cancelClimbAndStow() {
        return elevatorArmIntakeHandler.moveToStowPositions()
            .andThen(climber.moveToStowPosition());
    }

    public Command initClimber() {
        return Commands.sequence(
            elevatorArmIntakeHandler.moveIntakeDown(),
            Commands.waitUntil(intake::canRunRollers),
            climber.moveToStowPosition()            
        );
    }

    public Command stopRollers() {
        return Commands.sequence(
            intake.stopIntaking(),
            transfer.stopRollers(),
            endEffector.stopRollers()
        );
    }

    public Command coastAll() {
        return elevatorArmIntakeHandler.coastAll()
            .andThen(climber.coastArm())
            .ignoringDisable(true);
    }
}