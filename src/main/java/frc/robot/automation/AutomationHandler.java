package frc.robot.automation;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.KrakenSwerve;

public class AutomationHandler {
    private static enum GamePieceState {
        GAMEPIECES_NOT_READY,
        HAS_CORAL,
        HAS_ALGAE
    }

    private final Compositions compositions;

    private final Indexer indexer;
    private final EndEffector endEffector;
    private final Intake intake;

    public AutomationHandler(Compositions compositions, Arm arm, Elevator elevator, EndEffector endEffector, Indexer indexer, Intake intake, KrakenSwerve swerve, ButtonBoardHandler buttonBoard) {
        this.indexer = indexer;
        this.endEffector = endEffector;
        this.intake = intake;
        this.compositions = compositions;
    }

    public Command automationResumeCommand() {
        return new SelectCommand<GamePieceState>(
            Map.ofEntries(
                Map.entry(GamePieceState.GAMEPIECES_NOT_READY, mainCyclingCommand()),
                Map.entry(GamePieceState.HAS_CORAL, compositions.resumeCoralSequence().andThen(mainCyclingCommand()))
                /* Algae mode must be scheduled seperately from coral to avoid requirement conflicts */
               // Map.entry(GamePieceState.HAS_ALGAE, new ScheduleCommand(compositions.scoreAlgaeSequence()).andThen(mainCyclingCommand()))
            ),
            this::getGamePieceState
        );
    }

    private Command mainCyclingCommand() {
        /* This is the main cycling command, so it's repeated */
        return compositions.noPieceSequence().repeatedly();
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