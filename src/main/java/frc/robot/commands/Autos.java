package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.automation.Compositions;
import frc.robot.automation.ButtonBoardHandler.ReefLevel;
import frc.robot.subsystems.swerve.KrakenSwerve;

public class Autos {
    public static Command left4CoralAuto(Compositions compositions) {
        return Commands.sequence(//TODO branch if coral is missed
            /* Score preload and init climber */
            compositions.alignToReefAndScoreAutoPreload(20, true, ReefLevel.L4, false)
                .alongWith(compositions.initClimberIfNeeded()),
            /* Get and score coral 2 */
            compositions.getCoralFromSourceThenScore(19, true, true, ReefLevel.L4),
            /* Get and score coral 3 */
            compositions.getCoralFromSourceThenScore(19, false, true, ReefLevel.L4),
            /* Get and score coral 4 */
            compositions.getCoralFromSourceThenScore(18, true, true, ReefLevel.L4)
        );
    }

    public static Command right4CoralAuto(Compositions compositions) {
        return Commands.sequence(
            /* Score preload and init climber */
            compositions.alignToReefAndScoreAutoPreload(22, false, ReefLevel.L4, false)
                .alongWith(compositions.initClimberIfNeeded()),
            /* Get and score coral 2 */
            compositions.getCoralFromSourceThenScore(17, false, false, ReefLevel.L4),
            /* Get and score coral 3 */
            compositions.getCoralFromSourceThenScore(17, true, false, ReefLevel.L4),
            /* Get and score coral 4 */
            compositions.getCoralFromSourceThenScore(18, false, false, ReefLevel.L4)
        );
    }

    public static Command center1Coral1AlgaeAuto(Compositions compositions, KrakenSwerve swerve) {
        return Commands.sequence(
            Commands.waitSeconds(1),
            compositions.alignToReefAndScoreAutoPreload(21, true, ReefLevel.L4, true),
            compositions.intakeNearestAlgaeFromReef(() -> false),
            compositions.scoreAlgaeInNet(() -> false),
            Commands.waitSeconds(3.0)//TODO make this drive to a pose
                .deadlineFor(new PullAway(swerve, true, -1.0).asProxy())
        );
    }
}
