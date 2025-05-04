package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.automation.Compositions;
import frc.robot.automation.ElevatorArmIntakeHandler;
import frc.robot.automation.PathfindingHandler;
import frc.robot.automation.ButtonBoardHandler.AlgaeScorePath;
import frc.robot.automation.ButtonBoardHandler.ReefLevel;
import frc.robot.automation.ButtonBoardHandler.ReefPath;
import frc.robot.automation.ButtonBoardHandler.SourcePath;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.swerve.KrakenSwerve;
import frc.robot.utils.ElasticUtil;

public class Autos {
    private static final Trigger moveElevatorTrigger = new EventTrigger("ElevatorAuto")
        .onTrue(Commands.runOnce(() -> shouldMoveElevator = true));

    private static boolean shouldMoveElevator = false;

    public static Command fourCoralLeft(ElevatorArmIntakeHandler elevatorArmIntakeHandler, EndEffector endEffector, KrakenSwerve swerve, Compositions compositions, boolean mirrored) {
        PathPlannerPath score1;
        PathPlannerPath get2;
        PathPlannerPath score2;
        PathPlannerPath get3;
        PathPlannerPath score3;
        PathPlannerPath get4;
        PathPlannerPath score4;

        try {
            score1 = PathPlannerPath.fromPathFile("Score Coral 1");
            get2 = PathPlannerPath.fromPathFile("Get Coral 2");
            score2 = PathPlannerPath.fromPathFile("Score Coral 2");
            get3 = PathPlannerPath.fromPathFile("Get Coral 3");
            score3 = PathPlannerPath.fromPathFile("Score Coral 3");
            get4 = PathPlannerPath.fromPathFile("Get Coral 4");
            score4 = PathPlannerPath.fromPathFile("Score Coral 4 L4");

            if (mirrored) {
                score1 = score1.mirrorPath();
                get2 = get2.mirrorPath();
                score2 = score2.mirrorPath();
                get3 = get3.mirrorPath();
                score3 = score3.mirrorPath();
                get4 = get4.mirrorPath();
                score4 = score4.mirrorPath();
            }
        } catch (Exception e) {
            ElasticUtil.sendError("Could not load auto path!", "Auto will not work!");

            return Commands.none();
        }

        return Commands.sequence(
            /* Score preload */
            scorePreloadCoral(score1, elevatorArmIntakeHandler, endEffector),
            /* Get and score coral 2 */
            getAndScoreCoral(get2, score2, elevatorArmIntakeHandler, compositions, endEffector),
            /* Get and score coral 3 */
            getAndScoreCoral(get3, score3, elevatorArmIntakeHandler, compositions, endEffector),
            /* Get and score coral 4 */
            //getAndScoreCoral(get4, score4, elevatorArmIntakeHandler, compositions, endEffector),
            /* Return to stow positions */
            elevatorArmIntakeHandler.moveToStowPositions()
        );
    }

    public static Command oneCoralTwoAlgaeCenter(ElevatorArmIntakeHandler elevatorArmIntakeHandler, EndEffector endEffector, KrakenSwerve swerve, Compositions compositions) {
        PathPlannerPath scoreCoral; 
        PathPlannerPath pullAway1; 
        PathPlannerPath scoreAlgae1; 
        PathPlannerPath getAlgae2; 
        PathPlannerPath scoreAlgae2;
        PathPlannerPath pullAwayFromBarge;

        try {
            scoreCoral = PathPlannerPath.fromPathFile("Score Center");
            pullAway1 = PathPlannerPath.fromPathFile("Pull Away 1");
            scoreAlgae1 = PathPlannerPath.fromPathFile("Get And Score Algae 1");
            getAlgae2 = PathPlannerPath.fromPathFile("Get Algae 2");
            scoreAlgae2 = PathPlannerPath.fromPathFile("Score Algae 2");
            pullAwayFromBarge = PathPlannerPath.fromPathFile("Pull Away From Barge");
        } catch (Exception e) {
            ElasticUtil.sendError("Could not load auto path!", "Auto will not work!");

            return Commands.none();
        }

        return Commands.sequence( 
            scorePreloadCoral(scoreCoral, elevatorArmIntakeHandler, endEffector),
            getAndScoreAlgaeFromCoral(scoreAlgae1, pullAway1, elevatorArmIntakeHandler, compositions, endEffector),
            getAndScoreAlgaeFromBarge(getAlgae2, scoreAlgae2, elevatorArmIntakeHandler, compositions, endEffector, true),
            AutoBuilder.followPath(pullAwayFromBarge)
                .alongWith(elevatorArmIntakeHandler.moveToStowPositions())
        );
    }

    private static Command getAndScoreAlgaeFromCoral(PathPlannerPath getAndScoreAlgaePath, PathPlannerPath pullAwayPath, ElevatorArmIntakeHandler elevatorArmIntakeHandler, Compositions compositions, EndEffector endEffector) {
        return 
            /* Follow get algae and score algae paths */
            Commands.sequence(
                AutoBuilder.followPath(pullAwayPath)
                    .alongWith(elevatorArmIntakeHandler.moveToStowPositions()),
                elevatorArmIntakeHandler.prepareForAlgaeReefIntaking(() -> true),
                AutoBuilder.followPath(getAndScoreAlgaePath)
                    .alongWith(
                        endEffector.pickupAlgae().asProxy(),
                        Commands.waitUntil(moveElevatorTrigger)
                            .andThen(elevatorArmIntakeHandler.prepareForAlgaeScoring(() -> AlgaeScorePath.TO_NET))
                    ),
                /* Score the algae when the paths finish and everything is at setpoint */
                endEffector.outtakeNetAlgae().asProxy(),
                Commands.runOnce(() -> shouldMoveElevator = false)
            );
    }

    private static Command getAndScoreAlgaeFromBarge(PathPlannerPath getPath, PathPlannerPath scorePath, ElevatorArmIntakeHandler elevatorArmIntakeHandler, Compositions compositions, EndEffector endEffector, boolean highAlgae) {
        return 
            /* Follow get algae and score algae paths */
            Commands.sequence(
                AutoBuilder.followPath(getPath)
                    .alongWith(
                        elevatorArmIntakeHandler.moveToStowPositions()
                            .andThen(elevatorArmIntakeHandler.prepareForAlgaeReefIntaking(() -> !highAlgae)),
                        endEffector.pickupAlgae().asProxy()
                    ),
                AutoBuilder.followPath(scorePath)
                    .alongWith(
                        Commands.waitUntil(moveElevatorTrigger),
                        elevatorArmIntakeHandler.prepareForAlgaeScoring(() -> AlgaeScorePath.TO_NET)
                    ),
                /* Score the algae when the paths finish and everything is at setpoint */
                endEffector.outtakeNetAlgae().asProxy(),
                Commands.runOnce(() -> shouldMoveElevator = false)
            );
    }

    private static Command getAndScoreCoral(PathPlannerPath getCoralPath, PathPlannerPath scoreCoralPath, ElevatorArmIntakeHandler elevatorArmIntakeHandler, Compositions compositions, EndEffector endEffector) {
        return 
            /* Follow get coral and score coral paths */
            Commands.sequence(
                AutoBuilder.followPath(getCoralPath),
                AutoBuilder.followPath(scoreCoralPath)
            )
            /* At the same time, intake coral and prepare for scoring */
            .alongWith(
                Commands.sequence(
                    elevatorArmIntakeHandler.moveIntakeDown(),//Make sure the intake is down in time
                    compositions.intakeCoralToEndEffector(true),
                    elevatorArmIntakeHandler.prepareForAutoCoralScoring(() -> shouldMoveElevator),
                    ElasticUtil.sendInfoCommand("Prepared for coral scoring in auto")
                )
            )
            /* Score the coral when the paths finish and everything is at setpoint */
            .andThen(
                Commands.waitSeconds(0.25),//TODO d
                endEffector.scoreCoral(() -> ReefLevel.L4).asProxy(),
                Commands.runOnce(() -> shouldMoveElevator = false)
            );
    }

    private static Command scorePreloadCoral(PathPlannerPath scoreCoralPath, ElevatorArmIntakeHandler elevatorArmIntakeHandler, EndEffector endEffector) {
        return Commands.sequence(
            AutoBuilder.followPath(scoreCoralPath)
                .alongWith(elevatorArmIntakeHandler.prepareForAutoCoralScoring(() -> shouldMoveElevator))
                    .deadlineFor(endEffector.holdCoral().asProxy()),
            Commands.waitSeconds(0.25),//TODO d
            endEffector.scoreCoral(() -> ReefLevel.L4).asProxy(),
            Commands.runOnce(() -> shouldMoveElevator = false)
        );
    }

    public static Command dynamicCoralAuto(Compositions compositions, ElevatorArmIntakeHandler elevatorArmIntakeHandler) {
        return Commands.sequence(
            /* Score preload, and move to source area while preparing for intaking */
            compositions.alignToReefAndScoreFromPreset(ReefPath.TO_4L, ReefLevel.L4),
            PathfindingHandler.pathToSource(() -> SourcePath.TO_LSOURCE)
                .alongWith(elevatorArmIntakeHandler.moveToIntakePosition()),
            /* Get coral 2 */
            compositions.intakeNearestCoral(true),
            /* Score coral 2 */
            compositions.alignToReefAndScoreFromPreset(ReefPath.TO_2L, ReefLevel.L4),
            /* Get coral 3 */
            compositions.intakeNearestCoral(true),
            /* Score coral 3 */
            compositions.alignToReefAndScoreFromPreset(ReefPath.TO_2R, ReefLevel.L4),
            /* Stow */
            elevatorArmIntakeHandler.moveToStowPositions()
        );
    }
}
