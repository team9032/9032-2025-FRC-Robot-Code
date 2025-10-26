package frc.robot.automation;

import static frc.robot.Constants.PathFollowingConstants.kAlgaeIntakeWait;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.automation.ButtonBoardHandler.ReefLevel;
import frc.robot.commands.PullAway;
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

    public Command getCoralFromSourceThenScore(int reefTagID, boolean isLeftBranch, boolean isLeftSource, ReefLevel reefLevel) {
        return Commands.sequence(
            Commands.print("Getting coral from source"),
            PathfindingHandler.pathToSourceThenCoral(swerve, isLeftSource),
            Commands.print("Pathing to reef branch after source coral"),
            PathfindingHandler.pathToReefBranch(reefTagID, swerve, isLeftBranch)
        )                
        .alongWith(
            Commands.sequence(
                intakeCoralToEndEffector(false),
                /* Moves the elevator and arm when the robot is close enough to the reef */
                elevatorArmIntakeHandler.prepareForBranchCoralScoringFromCradle(() -> reefLevel),
                Commands.waitUntil(() -> elevatorArmIntakeHandler.readyToScoreCoralOnBranch(reefLevel))
            )
        )
        .andThen(placeCoralAndPullAway(() -> reefLevel, false));
    }

    public Command alignToReefAndScoreAutoPreload(int reefTagID, boolean isLeftBranch, ReefLevel reefLevel, boolean endPullAway) {
        return Commands.sequence(
            Commands.print("Aligning to reef and scoring preload"),
            endEffector.startRollersForPickup(),
            PathfindingHandler.pathToReefBranch(reefTagID, swerve, isLeftBranch).asProxy()
                /* Moves the elevator and arm when the robot is close enough to the reef */
                .alongWith(
                    elevatorArmIntakeHandler.moveToStowPositions()
                        .andThen(
                            Commands.waitUntil(() -> FieldUtil.shouldPrepareToScoreCoral(swerve.getLocalization())),
                            elevatorArmIntakeHandler.prepareForBranchCoralScoring(() -> reefLevel)
                        )
                ),
            Commands.waitUntil(() -> elevatorArmIntakeHandler.readyToScoreCoralOnBranch(reefLevel)),
            placeCoralAndPullAway(() -> reefLevel, endPullAway)
        );
    }

    private Command placeCoralAndPullAway(Supplier<ReefLevel> reefLevelSup, boolean endPullAway) {
        return placeCoralOnBranch(reefLevelSup)//TODO test pull away
            .alongWith(
                Commands.sequence(
                    Commands.waitUntil(() -> elevatorArmIntakeHandler.readyToPullAway(reefLevelSup.get())),
                    Commands.waitUntil(() -> FieldUtil.endEffectorCanClearReef(swerve.getLocalization()))
                        .deadlineFor(new PullAway(swerve, endPullAway).asProxy())   
                )
            );
    }

    public Command alignToReefAndScore(boolean isLeftBranch, Supplier<ReefLevel> reefLevelSup, BooleanSupplier shouldInterrupt, Command rumbleCommand) {
        return Commands.sequence(
            Commands.print("Aligning to reef and scoring"),
            endEffector.startRollersForPickup(),
            PathfindingHandler.pathToClosestReefBranch(swerve, isLeftBranch).asProxy()
                /* Moves the elevator and arm when the robot is close enough to the reef */
                .alongWith(
                    elevatorArmIntakeHandler.moveToStowPositions()
                        .andThen(
                            Commands.waitUntil(() -> FieldUtil.shouldPrepareToScoreCoral(swerve.getLocalization())),
                            elevatorArmIntakeHandler.prepareForBranchCoralScoring(reefLevelSup)
                        ) 
                ),
            Commands.waitUntil(() -> elevatorArmIntakeHandler.readyToScoreCoralOnBranch(reefLevelSup.get())),
            placeCoralAndPullAway(reefLevelSup, true),
            rumbleCommand
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
        );
    }

    public Command placeCoralOnBranch(Supplier<ReefLevel> reefLevelSup) {
        return elevatorArmIntakeHandler.moveArmToReefBranchScorePos(reefLevelSup)
            .alongWith(endEffector.placeCoralOnBranch(reefLevelSup));
    } 

    public Command scoreL1(boolean isHigh) {
        return Commands.sequence(
            Commands.print("Scoring L1 - is high " + isHigh),
            elevatorArmIntakeHandler.prepareForHighL1()
                .onlyIf(() -> isHigh),
            endEffector.placeCoralInTrough(),
            Commands.waitUntil(() -> FieldUtil.endEffectorCanClearReef(swerve.getLocalization())),
            elevatorArmIntakeHandler.moveToIntakePosition()   
        )
        .onlyIf(endEffector::hasCoral);
    }

    public Command intakeNearestCoral() {
        return Commands.sequence(
            Commands.print("Started intaking nearest coral"),
            PathfindingHandler.pathToNearestCoral(swerve)
                .alongWith(intakeCoralToEndEffector())
        );
    }

    public Command pulseIntake() {
        return Commands.sequence(
            Commands.print("Pulsing intake"),
            cancelIntake(),
            Commands.waitSeconds(0.08),
            intakeCoralToEndEffector()
        )
        .onlyIf(() -> !endEffector.hasCoral() && !endEffector.hasAlgae());
    }

    public Command intakeCoralToCradle() {
        return Commands.sequence(
            Commands.print("Started intaking to cradle"),
            intake.moveToGround(),
            Commands.waitUntil(intake::canRunRollers),
            intake.intakeCoral(),
            transfer.receiveCoralFromIntake(),
            intake.stopIntaking()
        );
    }

    public Command intakeCoralToEndEffector() {
        return intakeCoralToEndEffector(true);
    }

    public Command intakeCoralToEndEffector(boolean moveToStow) {
        return Commands.sequence(
            Commands.print("Started intaking"),
            Commands.waitUntil(() -> FieldUtil.endEffectorCanClearReef(swerve.getLocalization()))
                .andThen(elevatorArmIntakeHandler.moveToIntakePosition())//Don't hit the reef when moving to intake position
                    .alongWith(
                        Commands.sequence(
                            intake.moveToGround(),
                            Commands.waitUntil(intake::canRunRollers),
                            intake.intakeCoral(),
                            endEffector.startRollersForPickup(),
                            transfer.receiveCoralFromIntakeForPickup()
                        )
                    ),
            intake.stopIntaking(),
            elevatorArmIntakeHandler.moveToCoralCradlePosition()
                .alongWith(endEffector.pickupCoralFromCradle()),
            transfer.stopRollers(),
            Commands.waitUntil(() -> FieldUtil.endEffectorCanClearReef(swerve.getLocalization())),//Don't hit the reef when moving to stow
            /* Prepare for L1 early instead of stowing */
            Commands.either(
                elevatorArmIntakeHandler.prepareForL1(),
                elevatorArmIntakeHandler.moveToStowPositions(),
                () -> buttonBoardHandler.getSelectedReefLevel().equals(ReefLevel.L1)
            )
            .onlyIf(() -> moveToStow)
        )
        .onlyIf(() -> !endEffector.hasCoral() && !endEffector.hasAlgae());
    }

    public Command cancelIntake() {
        return Commands.sequence(
            Commands.print("Canceled intaking"),
            elevatorArmIntakeHandler.moveIntakeUp(),
            intake.ejectCoral()
                .alongWith(transfer.eject())
        );
    }       

    public Command ejectIntake() {
        return Commands.sequence(
            Commands.print("Ejected intake"),
            stopRollers(),
            intake.ejectCoral()
        );
    }

    public Command intakeNearestAlgaeFromReef(BooleanSupplier shouldInterrupt) {
        return Commands.sequence(
            Commands.print("Intaking algae from the reef"),
            stopRollers(),
            elevatorArmIntakeHandler.prepareForAlgaeReefIntaking(() -> FieldUtil.isClosestReefLocationHighAlgae(swerve.getLocalization())),
            PathfindingHandler.pathToClosestReefAlgaeIntake(swerve).asProxy()
                .alongWith(endEffector.intakeAlgae()),
            Commands.waitUntil(() -> FieldUtil.endEffectorWithAlgaeCanClearReef(swerve.getLocalization()))
                .deadlineFor(
                    Commands.waitSeconds(kAlgaeIntakeWait)
                    .andThen(new PullAway(swerve, true).asProxy())
                )
        )
        .until(shouldInterrupt)
            .andThen(
                Commands.either(
                    Commands.waitUntil(() -> FieldUtil.endEffectorWithAlgaeCanClearReef(swerve.getLocalization())),
                    endEffector.stopRollers(),
                    endEffector::hasAlgae
                ),
                elevatorArmIntakeHandler.moveToStowPositions()
            )
        .onlyIf(() -> !endEffector.hasCoral() && !endEffector.hasAlgae());
    }

    public Command scoreAlgaeInNet(BooleanSupplier shouldInterrupt) {
        return Commands.sequence(
            Commands.print("Scoring algae in the net"),
            PathfindingHandler.pathToBarge(swerve).asProxy()
                .alongWith(
                    Commands.sequence(
                        elevatorArmIntakeHandler.moveToStowPositions(),
                        Commands.waitUntil(() -> FieldUtil.shouldPrepareToScoreNetAlgae(swerve.getLocalization())),
                        elevatorArmIntakeHandler.prepareForNetAlgaeScoring()
                    )
                ),
            endEffector.outtakeNetAlgae()
        )
        .until(shouldInterrupt)
        .andThen(elevatorArmIntakeHandler.moveToStowPositionsFromNet())
        .onlyIf(endEffector::hasAlgae);
    }

    public Command intakeGroundAlgae() {
        return Commands.sequence(
            Commands.print("Intaking ground algae"),
            elevatorArmIntakeHandler.prepareForAlgaeGroundIntaking(),
            endEffector.intakeAlgae(),
            elevatorArmIntakeHandler.moveToStowPositions()  
        )
        .onlyIf(() -> !endEffector.hasAlgae() && !endEffector.hasCoral());
    }

    public Command climb() {
        return Commands.sequence(
            Commands.print("Climbing"),
            stopRollers(),
            elevatorArmIntakeHandler.prepareForClimbing(),
            climber.intakeCageAndClimb()
        );
    }

    public Command cancelClimbAndStow() {
        return Commands.sequence(
            Commands.print("Cancelling climb and stowing"),
            elevatorArmIntakeHandler.moveToStowPositions(),
            climber.moveToStowPosition()
        );
    }

    public Command initClimberIfNeeded() {
        return Commands.sequence(
            Commands.print("Initing climber"),
            elevatorArmIntakeHandler.moveIntakeDown(),
            Commands.waitUntil(intake::canRunRollers),
            climber.moveToStowPosition()            
        )
        .onlyIf(() -> !climber.atStow());
    }

    public Command stopRollers() {
        return Commands.sequence(
            intake.stopIntaking(),
            transfer.stopRollers(),
            endEffector.stopRollers()
        );
    }

    public Command coastAll() {
        return Commands.sequence(
            elevatorArmIntakeHandler.coastAll(),
            climber.coastArm(),
            ElasticUtil.sendInfoCommand("Coasted All Motors")
        )
        .ignoringDisable(true);
    }
}