package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.automation.Compositions;
import frc.robot.automation.ElevatorArmIntakeHandler;
import frc.robot.automation.PathfindingHandler;
import frc.robot.automation.ButtonBoardHandler.ReefLevel;

public class Autos {
    public static Command dynamicCoralAuto(Compositions compositions, ElevatorArmIntakeHandler elevatorArmIntakeHandler) {
        return Commands.sequence(
             /* Score preload, and move to source area while preparing for intaking */
            compositions.alignToReefAndScoreFromPreset(20, true, ReefLevel.L4)
                .alongWith(compositions.initClimberIfNeeded()),
            PathfindingHandler.pathToSource(true)
        // .alongWith(elevatorArmIntakeHandler.moveToIntakePosition())
        //     /* Get coral 2 */
        //     compositions.intakeNearestCoral(true),
        //     /* Score coral 2 */
        //     compositions.alignToReefAndScoreFromPreset(ReefPath.TO_2L, ReefLevel.L4),
        //     /* Get coral 3 */
        //     compositions.intakeNearestCoral(true),
        //     /* Score coral 3 */
        //     compositions.alignToReefAndScoreFromPreset(ReefPath.TO_2R, ReefLevel.L4),
        //     /* Stow */
        //     elevatorArmIntakeHandler.moveToStowPositions()
        );
    }
}
