package frc.robot.automation;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.swerve.KrakenSwerve;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.State;

import static frc.robot.Constants.AutomationConstants.*;

import java.util.Map;

public class ButtonBoardHandler {
    private final CommandXboxController buttonBoardController1 = new CommandXboxController(kButtonBoardPort1);
    private final CommandXboxController buttonBoardController2 = new CommandXboxController(kButtonBoardPort2);
    private final CommandXboxController buttonBoardController3 = new CommandXboxController(kButtonBoardPort3);

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable buttonTable = inst.getTable("ButtonTable");

    private final BooleanPublisher toLSourcePub = buttonTable.getBooleanTopic("toLSource").publish();
    private final BooleanPublisher toBargePub = buttonTable.getBooleanTopic("toBarge").publish();
    private final BooleanPublisher toRSourcePub = buttonTable.getBooleanTopic("toRSource").publish();
    private final BooleanPublisher toProcessorPub = buttonTable.getBooleanTopic("toProcessor").publish();
    private final BooleanPublisher to1LPub = buttonTable.getBooleanTopic("to1L").publish();
    private final BooleanPublisher to1RPub = buttonTable.getBooleanTopic("to1R").publish();
    private final BooleanPublisher to2LPub = buttonTable.getBooleanTopic("to2L").publish();
    private final BooleanPublisher to2RPub = buttonTable.getBooleanTopic("to2R").publish();
    private final BooleanPublisher to3LPub = buttonTable.getBooleanTopic("to3L").publish();
    private final BooleanPublisher to3RPub = buttonTable.getBooleanTopic("to3R").publish();
    private final BooleanPublisher to4LPub = buttonTable.getBooleanTopic("to4L").publish();
    private final BooleanPublisher to4RPub = buttonTable.getBooleanTopic("to4R").publish();
    private final BooleanPublisher to5LPub = buttonTable.getBooleanTopic("to5L").publish();
    private final BooleanPublisher to5RPub = buttonTable.getBooleanTopic("to5R").publish();
    private final BooleanPublisher to6LPub = buttonTable.getBooleanTopic("to6L").publish();
    private final BooleanPublisher to6RPub = buttonTable.getBooleanTopic("to6R").publish();

    private final BooleanPublisher toTroughPub = buttonTable.getBooleanTopic("toTrough").publish();
    private final BooleanPublisher toLevel1Pub = buttonTable.getBooleanTopic("toLevel1").publish();
    private final BooleanPublisher toLevel2Pub = buttonTable.getBooleanTopic("toLevel2").publish();
    private final BooleanPublisher toLevel3Pub = buttonTable.getBooleanTopic("toLevel3").publish();
    private final BooleanPublisher algaeModePub = buttonTable.getBooleanTopic("algaeToggle").publish();
    private final BooleanPublisher coralModePub = buttonTable.getBooleanTopic("automaticMode").publish();

    private final Trigger toLSource = buttonBoardController1.button(1);
    private final Trigger toBarge = buttonBoardController1.button(4);
    private final Trigger toRSource = buttonBoardController2.button(2);
    private final Trigger toProcessor = buttonBoardController2.button(11);
    
    private final Trigger to1L = buttonBoardController1.button(6);
    private final Trigger to1R = buttonBoardController1.button(5);
    private final Trigger to2L = buttonBoardController1.button(8);
    private final Trigger to2R = buttonBoardController1.button(7);
    private final Trigger to3L = buttonBoardController1.button(10);
    private final Trigger to3R = buttonBoardController1.button(11);
    private final Trigger to4L = buttonBoardController1.button(12);
    private final Trigger to4R = buttonBoardController2.button(1);
    private final Trigger to5L = buttonBoardController2.button(3);
    private final Trigger to5R = buttonBoardController2.button(4);
    private final Trigger to6L = buttonBoardController2.button(6);
    private final Trigger to6R = buttonBoardController2.button(7);

    private final Trigger toTrough = buttonBoardController2.button(5);
    private final Trigger toLevel1 = buttonBoardController2.button(9);
    private final Trigger toLevel2 = buttonBoardController2.button(8);
    private final Trigger toLevel3 = buttonBoardController2.button(10);

    private final Trigger enableAlgaeMode = buttonBoardController1.button(9);
    private final Trigger enableCoralMode = buttonBoardController1.button(3);
    private final Trigger autoIntake = buttonBoardController1.button(2);
     
    public final Trigger manual1 = buttonBoardController3.button(5);
    public final Trigger manual2 = buttonBoardController3.button(4);
    public final Trigger manual3 = buttonBoardController3.button(3);
    public final Trigger manual6 = buttonBoardController3.button(11);
    public final Trigger manual7 = buttonBoardController3.button(7);
    public final Trigger manual8 = buttonBoardController3.button(6);
    public final Trigger manual9 = buttonBoardController3.button(2);
    public final Trigger manual10 = buttonBoardController3.button(1);
    public final Trigger manual11 = buttonBoardController2.button(12);
    public final Trigger manual12 = buttonBoardController3.button(12);
    public final Trigger manual13 = buttonBoardController3.button(10);
    public final Trigger manual14 = buttonBoardController3.button(9);
    public final Trigger manual15 = buttonBoardController3.button(8);

    private final LED led;
    
    public ButtonBoardHandler(LED led) {
        this.led = led;

        bindButtons();
    }

    private static enum ReefPath {
        NONE,
        TO_1L,
        TO_1R,
        TO_2L,
        TO_2R,
        TO_3L,
        TO_3R,
        TO_4L,
        TO_4R,
        TO_5L,
        TO_5R,
        TO_6L,
        TO_6R
    }

    private ReefPath reefPathTarget = ReefPath.NONE;

    private static enum AlgaeScorePath {
        NONE,
        TO_BARGE,
        TO_PROCESSOR
    }

    private AlgaeScorePath algaeScorePathTarget = AlgaeScorePath.TO_BARGE;

    private static enum SourcePath {
        NONE,
        TO_LSOURCE,
        TO_RSOURCE
    }

    private SourcePath sourcePathTarget = SourcePath.TO_LSOURCE;

    private static enum ReefLevel {
        NONE,
        TO_TROUGH,
        TO_LEVEL1,
        TO_LEVEL2,
        TO_LEVEL3
    }

    private ReefLevel reefLevelTarget = ReefLevel.TO_LEVEL3;

    private void bindButtons() {
        toLSource.onTrue(Commands.runOnce(() -> sourcePathTarget = SourcePath.TO_LSOURCE));
        toBarge.onTrue(Commands.runOnce(() -> algaeScorePathTarget = AlgaeScorePath.TO_BARGE));
        toRSource.onTrue(Commands.runOnce(() -> sourcePathTarget = SourcePath.TO_RSOURCE));
        toProcessor.onTrue(Commands.runOnce(() -> algaeScorePathTarget = AlgaeScorePath.TO_PROCESSOR));
        to1L.onTrue(Commands.runOnce(() -> reefPathTarget = ReefPath.TO_1L).ignoringDisable(true));
        to1R.onTrue(Commands.runOnce(() -> reefPathTarget = ReefPath.TO_1R).ignoringDisable(true));
        to2L.onTrue(Commands.runOnce(() -> reefPathTarget = ReefPath.TO_2L).ignoringDisable(true));
        to2R.onTrue(Commands.runOnce(() -> reefPathTarget = ReefPath.TO_2R).ignoringDisable(true));
        to3L.onTrue(Commands.runOnce(() -> reefPathTarget = ReefPath.TO_3L).ignoringDisable(true));
        to3R.onTrue(Commands.runOnce(() -> reefPathTarget = ReefPath.TO_3R).ignoringDisable(true));
        to4L.onTrue(Commands.runOnce(() -> reefPathTarget = ReefPath.TO_4L).ignoringDisable(true));
        to4R.onTrue(Commands.runOnce(() -> reefPathTarget = ReefPath.TO_4R).ignoringDisable(true));
        to5L.onTrue(Commands.runOnce(() -> reefPathTarget = ReefPath.TO_5L).ignoringDisable(true));
        to5R.onTrue(Commands.runOnce(() -> reefPathTarget = ReefPath.TO_5R).ignoringDisable(true));
        to6L.onTrue(Commands.runOnce(() -> reefPathTarget = ReefPath.TO_6L).ignoringDisable(true));
        to6R.onTrue(Commands.runOnce(() -> reefPathTarget = ReefPath.TO_6R).ignoringDisable(true));

        toTrough.onTrue(Commands.runOnce(() -> reefLevelTarget = ReefLevel.TO_TROUGH));
        toLevel1.onTrue(Commands.runOnce(() -> reefLevelTarget = ReefLevel.TO_LEVEL1));
        toLevel2.onTrue(Commands.runOnce(() -> reefLevelTarget = ReefLevel.TO_LEVEL2));
        toLevel3.onTrue(Commands.runOnce(() -> reefLevelTarget = ReefLevel.TO_LEVEL3));
    }

    public Command scoreCoral(EndEffector endEffector) {
        return Commands.either(endEffector.placeCoralInTrough(), endEffector.placeCoral(), () -> reefLevelTarget.equals(ReefLevel.TO_TROUGH));
    }

    public Command followReefPath(KrakenSwerve swerve) {
        return new SelectCommand<ReefPath>(
            Map.ofEntries(
                Map.entry(ReefPath.NONE, Commands.none()),
                Map.entry(ReefPath.TO_1L, PathfindingHandler.pathTo1L(swerve)),
                Map.entry(ReefPath.TO_1R, PathfindingHandler.pathTo1R(swerve)),
                Map.entry(ReefPath.TO_2L, PathfindingHandler.pathTo2L(swerve)),
                Map.entry(ReefPath.TO_2R, PathfindingHandler.pathTo2R(swerve)),
                Map.entry(ReefPath.TO_3L, PathfindingHandler.pathTo3L(swerve)),
                Map.entry(ReefPath.TO_3R, PathfindingHandler.pathTo3R(swerve)),
                Map.entry(ReefPath.TO_4L, PathfindingHandler.pathTo4L(swerve)),
                Map.entry(ReefPath.TO_4R, PathfindingHandler.pathTo4R(swerve)),
                Map.entry(ReefPath.TO_5L, PathfindingHandler.pathTo5L(swerve)),
                Map.entry(ReefPath.TO_5R, PathfindingHandler.pathTo5R(swerve)),
                Map.entry(ReefPath.TO_6L, PathfindingHandler.pathTo6L(swerve)),
                Map.entry(ReefPath.TO_6R, PathfindingHandler.pathTo6R(swerve))
            ),
            () -> reefPathTarget
        );
    }

    // public Command followAlgaeIntakePath(KrakenSwerve swerve) {
    //     return new SelectCommand<ReefPath>(
    //         Map.ofEntries(
    //             Map.entry(ReefPath.NONE, Commands.none()),
    //             Map.entry(ReefPath.TO_1L, PathfindingHandler.pathTo1A(swerve)),
    //             Map.entry(ReefPath.TO_1R, PathfindingHandler.pathTo1A(swerve)),
    //             Map.entry(ReefPath.TO_2L, PathfindingHandler.pathTo2A(swerve)),
    //             Map.entry(ReefPath.TO_2R, PathfindingHandler.pathTo2A(swerve)),
    //             Map.entry(ReefPath.TO_3L, PathfindingHandler.pathTo3A(swerve)),
    //             Map.entry(ReefPath.TO_3R, PathfindingHandler.pathTo3A(swerve)),
    //             Map.entry(ReefPath.TO_4L, PathfindingHandler.pathTo4A(swerve)),
    //             Map.entry(ReefPath.TO_4R, PathfindingHandler.pathTo4A(swerve)),
    //             Map.entry(ReefPath.TO_5L, PathfindingHandler.pathTo5A(swerve)),
    //             Map.entry(ReefPath.TO_5R, PathfindingHandler.pathTo5A(swerve)),
    //             Map.entry(ReefPath.TO_6L, PathfindingHandler.pathTo6A(swerve)),
    //             Map.entry(ReefPath.TO_6R, PathfindingHandler.pathTo6A(swerve))
    //         ),
    //         () -> reefPathTarget
    //     );
    // }

    public Command followSourcePath() {
        return new SelectCommand<SourcePath>(
            Map.ofEntries(
                Map.entry(SourcePath.NONE, Commands.none()),
                Map.entry(SourcePath.TO_LSOURCE, PathfindingHandler.pathToLSource()),
                Map.entry(SourcePath.TO_RSOURCE, PathfindingHandler.pathToRSource())
            ),
            () -> sourcePathTarget
        );
    }

    public Command followAlgaeScorePath() {
        return new SelectCommand<AlgaeScorePath>(
            Map.ofEntries(
                Map.entry(AlgaeScorePath.NONE, Commands.none()),
                Map.entry(AlgaeScorePath.TO_BARGE, PathfindingHandler.pathToBarge()),
                Map.entry(AlgaeScorePath.TO_PROCESSOR, PathfindingHandler.pathToProcessor())
            ),
            () -> algaeScorePathTarget
        );
    }

    public Command moveArmToCoralTargetLevel(Arm arm) {
        return new SelectCommand<ReefLevel>(
            Map.ofEntries (
                Map.entry(ReefLevel.NONE, Commands.none()),
                Map.entry(ReefLevel.TO_TROUGH, arm.moveToTroughPos()),
                Map.entry(ReefLevel.TO_LEVEL1, arm.moveToLevel1Pos()),
                Map.entry(ReefLevel.TO_LEVEL2, arm.moveToLevel2Pos()),
                Map.entry(ReefLevel.TO_LEVEL3, arm.moveToLevel3Pos())
            ),
            () -> reefLevelTarget
        );
    }

    public Command moveArmToAlgaeIntakeTargetLevel(Arm arm) {
        /* Algae positions are based on the game manual */
        return new SelectCommand<ReefPath>(
            Map.ofEntries (
                Map.entry(ReefPath.NONE, Commands.none()),
                Map.entry(ReefPath.TO_1L, arm.moveToLowAlgaePos()),
                Map.entry(ReefPath.TO_1R, arm.moveToLowAlgaePos()),
                Map.entry(ReefPath.TO_2L, arm.moveToHighAlgaePos()),
                Map.entry(ReefPath.TO_2R, arm.moveToHighAlgaePos()),
                Map.entry(ReefPath.TO_3L, arm.moveToHighAlgaePos()),
                Map.entry(ReefPath.TO_3R, arm.moveToHighAlgaePos()),
                Map.entry(ReefPath.TO_4L, arm.moveToLowAlgaePos()),
                Map.entry(ReefPath.TO_4R, arm.moveToLowAlgaePos()),
                Map.entry(ReefPath.TO_5L, arm.moveToLowAlgaePos()),
                Map.entry(ReefPath.TO_5R, arm.moveToLowAlgaePos()),
                Map.entry(ReefPath.TO_6L, arm.moveToHighAlgaePos()),
                Map.entry(ReefPath.TO_6R, arm.moveToHighAlgaePos())
            ),
            () -> reefPathTarget
        );
    }

    public Command moveElevatorToAlgaeIntakeTargetLevel(Elevator elevator) {
        /* Algae positions are based on the game manual */
        return new SelectCommand<ReefPath>(
            Map.ofEntries (
                Map.entry(ReefPath.NONE, Commands.none()),
                Map.entry(ReefPath.TO_1L, elevator.moveToLowAlgaePosition()),
                Map.entry(ReefPath.TO_1R, elevator.moveToLowAlgaePosition()),
                Map.entry(ReefPath.TO_2L, elevator.moveToHighAlgaePosition()),
                Map.entry(ReefPath.TO_2R, elevator.moveToHighAlgaePosition()),
                Map.entry(ReefPath.TO_3L, elevator.moveToHighAlgaePosition()),
                Map.entry(ReefPath.TO_3R, elevator.moveToHighAlgaePosition()),
                Map.entry(ReefPath.TO_4L, elevator.moveToLowAlgaePosition()),
                Map.entry(ReefPath.TO_4R, elevator.moveToLowAlgaePosition()),
                Map.entry(ReefPath.TO_5L, elevator.moveToLowAlgaePosition()),
                Map.entry(ReefPath.TO_5R, elevator.moveToLowAlgaePosition()),
                Map.entry(ReefPath.TO_6L, elevator.moveToHighAlgaePosition()),
                Map.entry(ReefPath.TO_6R, elevator.moveToHighAlgaePosition())
            ),
            () -> reefPathTarget
        );
    }

    public Command moveElevatorToCoralTargetLevel(Elevator elevator) {
        return new SelectCommand<ReefLevel>(
            Map.ofEntries (
                Map.entry(ReefLevel.NONE, Commands.none()),
                Map.entry(ReefLevel.TO_TROUGH, elevator.moveToTroughPosition()),
                Map.entry(ReefLevel.TO_LEVEL1, elevator.moveToL1Position()),
                Map.entry(ReefLevel.TO_LEVEL2, elevator.moveToL2Position()),
                Map.entry(ReefLevel.TO_LEVEL3, elevator.moveToL3Position())
            ),
            () -> reefLevelTarget
        );
    }

    public Command moveArmToAlgaeScoreLevel(Arm arm) {
        return new SelectCommand<AlgaeScorePath>(
            Map.ofEntries(
                Map.entry(AlgaeScorePath.NONE, Commands.none()),
                Map.entry(AlgaeScorePath.TO_BARGE, arm.moveToNetPos()),
                Map.entry(AlgaeScorePath.TO_PROCESSOR, arm.moveToProcessorPos())
            ),
            () -> algaeScorePathTarget
        );
    }

    public Command moveElevatorToAlgaeScoreLevel(Elevator elevator) {
        return new SelectCommand<AlgaeScorePath>(
            Map.ofEntries(
                Map.entry(AlgaeScorePath.NONE, Commands.none()),
                Map.entry(AlgaeScorePath.TO_BARGE, elevator.moveToNetPosition()),
                Map.entry(AlgaeScorePath.TO_PROCESSOR, elevator.moveToProcessorPosition())
            ),
            () -> algaeScorePathTarget
        );
    }

    public Command scoreAlgae(EndEffector endEffector) {
        return new SelectCommand<AlgaeScorePath>(
            Map.ofEntries(
                Map.entry(AlgaeScorePath.NONE, Commands.none()),
                Map.entry(AlgaeScorePath.TO_BARGE, endEffector.outtakeNetAlgae()),
                Map.entry(AlgaeScorePath.TO_PROCESSOR, endEffector.outtakeProcessorAlgae())
            ),
            () -> algaeScorePathTarget
        ); 
    }

    public Trigger getEnableCoralModeTrigger() {
        return enableCoralMode;
    }

    public Trigger getEnableAlgaeModeTrigger() {
        return enableAlgaeMode;
    }

    public Command clearAlgaeTargets() {
        return Commands.runOnce(() -> algaeScorePathTarget = AlgaeScorePath.NONE);
    }

    public Command clearReefTargets() {
        return Commands.runOnce(() -> {
            reefLevelTarget = ReefLevel.NONE;
            reefPathTarget = ReefPath.NONE;
        });
    }

    public Trigger getAutoIntakeTrigger() {
        return autoIntake;
    }

    public void update(boolean coralModeEnabled, boolean algaeModeEnabled) {
        toLSourcePub.set(sourcePathTarget.equals(SourcePath.TO_LSOURCE));
        toBargePub.set(algaeScorePathTarget.equals(AlgaeScorePath.TO_BARGE));
        toRSourcePub.set(sourcePathTarget.equals(SourcePath.TO_RSOURCE));
        toProcessorPub.set(algaeScorePathTarget.equals(AlgaeScorePath.TO_PROCESSOR));

        to1LPub.set(reefPathTarget.equals(ReefPath.TO_1L));
        to2LPub.set(reefPathTarget.equals(ReefPath.TO_2L));
        to1RPub.set(reefPathTarget.equals(ReefPath.TO_1R));
        to2RPub.set(reefPathTarget.equals(ReefPath.TO_2R));
        to3LPub.set(reefPathTarget.equals(ReefPath.TO_3L));
        to3RPub.set(reefPathTarget.equals(ReefPath.TO_3R));
        to4LPub.set(reefPathTarget.equals(ReefPath.TO_4L));
        to4RPub.set(reefPathTarget.equals(ReefPath.TO_4R));
        to5LPub.set(reefPathTarget.equals(ReefPath.TO_5L));
        to5RPub.set(reefPathTarget.equals(ReefPath.TO_5R));
        to6LPub.set(reefPathTarget.equals(ReefPath.TO_6L));
        to6RPub.set(reefPathTarget.equals(ReefPath.TO_6R));

        toTroughPub.set(reefLevelTarget.equals(ReefLevel.TO_TROUGH));
        toLevel1Pub.set(reefLevelTarget.equals(ReefLevel.TO_LEVEL1));
        toLevel2Pub.set(reefLevelTarget.equals(ReefLevel.TO_LEVEL2));
        toLevel3Pub.set(reefLevelTarget.equals(ReefLevel.TO_LEVEL3));

        algaeModePub.set(algaeModeEnabled);
        coralModePub.set(coralModeEnabled);

        if (coralModeEnabled)
            updateCoralLEDs();

        else if (algaeModeEnabled)
            led.setState(State.ALGAE);
    }    

    private void updateCoralLEDs() {
        switch (reefLevelTarget) {
            case NONE:
            led.setState(State.ENABLED);
                break;

            case TO_LEVEL1:
            led.setState(State.L2);
                break;

            case TO_LEVEL2:
            led.setState(State.L3);
                break;

            case TO_LEVEL3:
            led.setState(State.L4);
                break;

            case TO_TROUGH:
            led.setState(State.L1);
                break;

            default:
            led.setState(State.ENABLED);
                break;
        }
    }

    public boolean hasQueues() {
        return reefLevelTarget != ReefLevel.NONE && reefPathTarget != ReefPath.NONE && sourcePathTarget != SourcePath.NONE;
    }

    public boolean readyToScoreCoral(Arm arm, Elevator elevator) {
        if (reefLevelTarget.equals(ReefLevel.TO_TROUGH))
            return arm.atTrough() && elevator.atTrough();

        else if (reefLevelTarget.equals(ReefLevel.TO_LEVEL1))
            return arm.atL1() && elevator.atL1();

        else if (reefLevelTarget.equals(ReefLevel.TO_LEVEL2))
            return arm.atL2() && elevator.atL2();

        else if (reefLevelTarget.equals(ReefLevel.TO_LEVEL3))
            return arm.atL3() && elevator.atL3();
        
        return false;
    }

    public boolean l4Selected() {
        return reefLevelTarget.equals(ReefLevel.TO_LEVEL3);
    }
}