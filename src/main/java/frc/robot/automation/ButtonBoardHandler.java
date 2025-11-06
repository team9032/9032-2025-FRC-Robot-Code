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

    private final BooleanPublisher toNetPub = buttonTable.getBooleanTopic("toBarge").publish();
    private final BooleanPublisher toProcessorPub = buttonTable.getBooleanTopic("toProcessor").publish();
    private final BooleanPublisher toL1Pub = buttonTable.getBooleanTopic("toTrough").publish();
    private final BooleanPublisher toL2Pub = buttonTable.getBooleanTopic("toLevel1").publish();
    private final BooleanPublisher toL3Pub = buttonTable.getBooleanTopic("toLevel2").publish();
    private final BooleanPublisher toL4Pub = buttonTable.getBooleanTopic("toLevel3").publish();

    private final Trigger toBarge = buttonBoardController1.button(4);
    private final Trigger toProcessor = buttonBoardController2.button(11);
    private final Trigger toL1 = buttonBoardController2.button(5);
    private final Trigger toL2 = buttonBoardController2.button(9);
    private final Trigger toL3 = buttonBoardController2.button(8);
    private final Trigger toL4 = buttonBoardController2.button(10);

    private final Trigger autoIntake = buttonBoardController1.button(2);
    public final Trigger manual1 = buttonBoardController3.button(5);
    public final Trigger manual2 = buttonBoardController3.button(4);
    public final Trigger manual3 = buttonBoardController3.button(3);
    public final Trigger manual4 = buttonBoardController3.button(9);
    public final Trigger manual5 = buttonBoardController3.button(8);
    public final Trigger manual6 = buttonBoardController3.button(11);
    public final Trigger manual7 = buttonBoardController3.button(7);
    public final Trigger manual8 = buttonBoardController3.button(6);
    public final Trigger manual9 = buttonBoardController3.button(2);
    public final Trigger manual10 = buttonBoardController3.button(1);
    public final Trigger manual11 = buttonBoardController2.button(12);
    public final Trigger manual12 = buttonBoardController3.button(12);
    public final Trigger manual13 = buttonBoardController3.button(10);
    
    public ButtonBoardHandler() {
        bindButtons();
    }

    public static enum AlgaeScorePath {
        NONE,
        TO_NET,
        TO_PROCESSOR
    }

    private AlgaeScorePath algaeScorePathTarget = AlgaeScorePath.TO_NET;

    public static enum ReefLevel {
        NONE,
        L1,
        L2,
        L3,
        L4
    }

    private ReefLevel reefLevelTarget = ReefLevel.L4;

    private void bindButtons() {
        toBarge.onTrue(Commands.runOnce(() -> algaeScorePathTarget = AlgaeScorePath.TO_NET).ignoringDisable(true));
        toProcessor.onTrue(Commands.runOnce(() -> algaeScorePathTarget = AlgaeScorePath.TO_PROCESSOR).ignoringDisable(true));

        toL1.onTrue(Commands.runOnce(() -> reefLevelTarget = ReefLevel.L1).ignoringDisable(true));
        toL2.onTrue(Commands.runOnce(() -> reefLevelTarget = ReefLevel.L2).ignoringDisable(true));
        toL3.onTrue(Commands.runOnce(() -> reefLevelTarget = ReefLevel.L3).ignoringDisable(true));
        toL4.onTrue(Commands.runOnce(() -> reefLevelTarget = ReefLevel.L4).ignoringDisable(true));
    }

    public ReefLevel getSelectedReefLevel() {
        return reefLevelTarget;
    }

    public AlgaeScorePath getSelectedAlgaeScorePath() {
        return algaeScorePathTarget;
    }

    public Command clearAlgaeTargets() {
        return Commands.runOnce(() -> algaeScorePathTarget = AlgaeScorePath.NONE);
    }

    public Command clearReefTargets() {
        return Commands.runOnce(() -> reefLevelTarget = ReefLevel.NONE);
    }

    public Trigger getAutoIntakeTrigger() {
        return autoIntake;
    }

    public void incrementReefLevel() {
        switch (reefLevelTarget) {
            case L1:
            reefLevelTarget = ReefLevel.L2;
                break;
            case L2:
            reefLevelTarget = ReefLevel.L3;
                break;
            case L3:
            reefLevelTarget = ReefLevel.L4;
                break;
            default:
                break;
        }
    }

    public void decrementReefLevel() {
        switch (reefLevelTarget) {
            case L2:
            reefLevelTarget = ReefLevel.L1;
                break;
            case L3:
            reefLevelTarget = ReefLevel.L2;
                break;
            case L4:
            reefLevelTarget = ReefLevel.L3;
                break;
            default:
                break;
        }
    }

    public void update() {
        toNetPub.set(algaeScorePathTarget.equals(AlgaeScorePath.TO_NET));
        toProcessorPub.set(algaeScorePathTarget.equals(AlgaeScorePath.TO_PROCESSOR));

        toL1Pub.set(reefLevelTarget.equals(ReefLevel.L1));
        toL2Pub.set(reefLevelTarget.equals(ReefLevel.L2));
        toL3Pub.set(reefLevelTarget.equals(ReefLevel.L3));
        toL4Pub.set(reefLevelTarget.equals(ReefLevel.L4));
    }    
}