package frc.robot.automation;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.subsystems.EndEffector;

public class AutomationHandler {
    private static enum GamePieceState {
        GAMEPIECES_NOT_READY,
        HAS_CORAL,
        HAS_ALGAE
    }

    private final Compositions compositions;

    private final EndEffector endEffector;

    public AutomationHandler(Compositions compositions, EndEffector endEffector, ButtonBoardHandler buttonBoard) {
        this.endEffector = endEffector;
        this.compositions = compositions;
    }

    public Command coralResumeCommand() {
        return new SelectCommand<GamePieceState>(
            Map.ofEntries(
                Map.entry(GamePieceState.GAMEPIECES_NOT_READY, compositions.driveToSource()),
                Map.entry(GamePieceState.HAS_CORAL, compositions.alignToReefAndScore().andThen(compositions.driveToSource())),
                Map.entry(GamePieceState.HAS_ALGAE, Commands.none())
            ),
            this::getGamePieceState
        );
    }

    public Command algaeResumeCommand() {
        return new SelectCommand<GamePieceState>(
            Map.ofEntries(
                Map.entry(GamePieceState.GAMEPIECES_NOT_READY, mainAlgaeCyclingCommand()),
                Map.entry(GamePieceState.HAS_CORAL, Commands.none()),
                Map.entry(GamePieceState.HAS_ALGAE, compositions.scoreAlgaeSequence().andThen(mainAlgaeCyclingCommand()))
            ),
            this::getGamePieceState
        );
    }

    private Command mainCoralCyclingCommand() {
        /* This is the main cycling command, so it's repeated */
        return compositions.driveToSource().asProxy()//Need to get coral
            .andThen(new ScheduleCommand(compositions.alignToReefAndScore()));
    }

    private Command mainAlgaeCyclingCommand() {
        /* This is the main cycling command, so it's repeated */
        return compositions.intakeAlgaeFromReef().repeatedly();
    }

    private GamePieceState getGamePieceState() {
        if (endEffector.hasCoral())
            return GamePieceState.HAS_CORAL;

        else if (endEffector.hasAlgae())
            return GamePieceState.HAS_ALGAE;

        else 
            return GamePieceState.GAMEPIECES_NOT_READY;
    }
}