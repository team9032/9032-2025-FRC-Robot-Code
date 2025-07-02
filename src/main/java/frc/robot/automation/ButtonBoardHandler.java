package frc.robot.automation;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.ButtonBoardConstants.*;

public class ButtonBoardHandler {
    private final CommandXboxController buttonBoardController1 = new CommandXboxController(kButtonBoardPort1);
    private final CommandXboxController buttonBoardController2 = new CommandXboxController(kButtonBoardPort2);
    private final CommandXboxController buttonBoardController3 = new CommandXboxController(kButtonBoardPort3);

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable buttonTable = inst.getTable("ButtonTable");

    // private final BooleanPublisher toLSourcePub = buttonTable.getBooleanTopic("toLSource").publish();
    // private final BooleanPublisher toNetPub = buttonTable.getBooleanTopic("toBarge").publish();
    // private final BooleanPublisher toRSourcePub = buttonTable.getBooleanTopic("toRSource").publish();
    // private final BooleanPublisher toProcessorPub = buttonTable.getBooleanTopic("toProcessor").publish();
    // private final BooleanPublisher to1LPub = buttonTable.getBooleanTopic("to1L").publish();
    // private final BooleanPublisher to1RPub = buttonTable.getBooleanTopic("to1R").publish();
    // private final BooleanPublisher to2LPub = buttonTable.getBooleanTopic("to2L").publish();
    // private final BooleanPublisher to2RPub = buttonTable.getBooleanTopic("to2R").publish();
    // private final BooleanPublisher to3LPub = buttonTable.getBooleanTopic("to3L").publish();
    // private final BooleanPublisher to3RPub = buttonTable.getBooleanTopic("to3R").publish();
    // private final BooleanPublisher to4LPub = buttonTable.getBooleanTopic("to4L").publish();
    // private final BooleanPublisher to4RPub = buttonTable.getBooleanTopic("to4R").publish();
    // private final BooleanPublisher to5LPub = buttonTable.getBooleanTopic("to5L").publish();
    // private final BooleanPublisher to5RPub = buttonTable.getBooleanTopic("to5R").publish();
    // private final BooleanPublisher to6LPub = buttonTable.getBooleanTopic("to6L").publish();
    // private final BooleanPublisher to6RPub = buttonTable.getBooleanTopic("to6R").publish();

    private final BooleanPublisher toL1Pub = buttonTable.getBooleanTopic("toTrough").publish();
    private final BooleanPublisher toL2Pub = buttonTable.getBooleanTopic("toLevel1").publish();
    private final BooleanPublisher toL3Pub = buttonTable.getBooleanTopic("toLevel2").publish();
    private final BooleanPublisher toL4Pub = buttonTable.getBooleanTopic("toLevel3").publish();

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

    private final Trigger toL1 = buttonBoardController2.button(5);
    private final Trigger toL2 = buttonBoardController2.button(9);
    private final Trigger toL3 = buttonBoardController2.button(8);
    private final Trigger toL4 = buttonBoardController2.button(10);

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
    
    public ButtonBoardHandler() {
        bindButtons();
    }

    public static enum ReefPath {
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

    public static enum AlgaeScorePath {
        NONE,
        TO_NET,
        TO_PROCESSOR
    }

    private AlgaeScorePath algaeScorePathTarget = AlgaeScorePath.TO_NET;

    public static enum SourcePath {
        NONE,
        TO_LSOURCE,
        TO_RSOURCE
    }

    private SourcePath sourcePathTarget = SourcePath.TO_LSOURCE;

    public static enum ReefLevel {
        NONE,
        L1,
        L2,
        L3,
        L4
    }

    private ReefLevel reefLevelTarget = ReefLevel.L4;

    private void bindButtons() {
        toLSource.onTrue(Commands.runOnce(() -> sourcePathTarget = SourcePath.TO_LSOURCE).ignoringDisable(true));
        toBarge.onTrue(Commands.runOnce(() -> algaeScorePathTarget = AlgaeScorePath.TO_NET).ignoringDisable(true));
        toRSource.onTrue(Commands.runOnce(() -> sourcePathTarget = SourcePath.TO_RSOURCE).ignoringDisable(true));
        toProcessor.onTrue(Commands.runOnce(() -> algaeScorePathTarget = AlgaeScorePath.TO_PROCESSOR).ignoringDisable(true));
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

        toL1.onTrue(Commands.runOnce(() -> reefLevelTarget = ReefLevel.L1).ignoringDisable(true));
        toL2.onTrue(Commands.runOnce(() -> reefLevelTarget = ReefLevel.L2).ignoringDisable(true));
        toL3.onTrue(Commands.runOnce(() -> reefLevelTarget = ReefLevel.L3).ignoringDisable(true));
        toL4.onTrue(Commands.runOnce(() -> reefLevelTarget = ReefLevel.L4).ignoringDisable(true));
    }

    public ReefLevel getSelectedReefLevel() {
        return reefLevelTarget;
    }

    public ReefPath getSelectedReefPath() {
        return reefPathTarget;
    }

    public SourcePath getSelectedSourcePath() {
        return sourcePathTarget;
    }

    public AlgaeScorePath getSelectedAlgaeScorePath() {
        return algaeScorePathTarget;
    }

    public boolean lowAlgaeSelected() {
        /* Algae positions are based on the game manual */
        return reefPathTarget.equals(ReefPath.TO_2L) || reefPathTarget.equals(ReefPath.TO_2R) 
            || reefPathTarget.equals(ReefPath.TO_4L) || reefPathTarget.equals(ReefPath.TO_4R) 
            || reefPathTarget.equals(ReefPath.TO_6L) || reefPathTarget.equals(ReefPath.TO_6R);
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

    public void update() {
        // toLSourcePub.set(sourcePathTarget.equals(SourcePath.TO_LSOURCE));
        // toNetPub.set(algaeScorePathTarget.equals(AlgaeScorePath.TO_NET));
        // toRSourcePub.set(sourcePathTarget.equals(SourcePath.TO_RSOURCE));
        // toProcessorPub.set(algaeScorePathTarget.equals(AlgaeScorePath.TO_PROCESSOR));

        // to1LPub.set(reefPathTarget.equals(ReefPath.TO_1L));
        // to2LPub.set(reefPathTarget.equals(ReefPath.TO_2L));
        // to1RPub.set(reefPathTarget.equals(ReefPath.TO_1R));
        // to2RPub.set(reefPathTarget.equals(ReefPath.TO_2R));
        // to3LPub.set(reefPathTarget.equals(ReefPath.TO_3L));
        // to3RPub.set(reefPathTarget.equals(ReefPath.TO_3R));
        // to4LPub.set(reefPathTarget.equals(ReefPath.TO_4L));
        // to4RPub.set(reefPathTarget.equals(ReefPath.TO_4R));
        // to5LPub.set(reefPathTarget.equals(ReefPath.TO_5L));
        // to5RPub.set(reefPathTarget.equals(ReefPath.TO_5R));
        // to6LPub.set(reefPathTarget.equals(ReefPath.TO_6L));
        // to6RPub.set(reefPathTarget.equals(ReefPath.TO_6R));

        toL1Pub.set(reefLevelTarget.equals(ReefLevel.L1));
        toL2Pub.set(reefLevelTarget.equals(ReefLevel.L2));
        toL3Pub.set(reefLevelTarget.equals(ReefLevel.L3));
        toL4Pub.set(reefLevelTarget.equals(ReefLevel.L4));
    }    

    public boolean backReefSegmentsSelected() {
        return reefPathTarget.equals(ReefPath.TO_3L) || reefPathTarget.equals(ReefPath.TO_3R) || 
            reefPathTarget.equals(ReefPath.TO_4L) || reefPathTarget.equals(ReefPath.TO_4R) || 
            reefPathTarget.equals(ReefPath.TO_5L) || reefPathTarget.equals(ReefPath.TO_5R);
    }
}