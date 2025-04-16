package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.automation.Compositions;
import frc.robot.automation.ElevatorArmIntakeHandler;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.KrakenSwerve;
import frc.robot.utils.ElasticUtil;

public class Autos {
    private static final Trigger moveElevatorTrigger = new EventTrigger("ElevatorAuto")
        .onTrue(Commands.runOnce(() -> shouldMoveElevator = true));

    private static boolean shouldMoveElevator = false;

    public static Command fourCoralLeft(Intake intake, ElevatorArmIntakeHandler elevatorArmIntakeHandler, EndEffector endEffector, KrakenSwerve swerve, Indexer indexer, Compositions compositions, boolean mirrored) {
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

    public static Command oneCoralTwoAlgaeCenter(ElevatorArmIntakeHandler elevatorArmIntakeHandler, EndEffector endEffector, KrakenSwerve swerve, Indexer indexer, Compositions compositions) {
        PathPlannerPath scoreCoral; 
        PathPlannerPath pullAway1; 
        PathPlannerPath scoreAlgae1; 
        PathPlannerPath getAlgae2; 
        PathPlannerPath scoreAlgae2;

        try {
            scoreCoral = PathPlannerPath.fromPathFile("Score Center");
            pullAway1 = PathPlannerPath.fromPathFile("Pull Away 1");
            scoreAlgae1 = PathPlannerPath.fromPathFile("Get And Score Algae 1");
            getAlgae2 = PathPlannerPath.fromPathFile("Get Algae 2");
            scoreAlgae2 = PathPlannerPath.fromPathFile("Score Algae 2");

        } catch (Exception e) {
            ElasticUtil.sendError("Could not load auto path!", "Auto will not work!");

            return Commands.none();
        }

        return scorePreloadCoral(scoreCoral, elevatorArmIntakeHandler, endEffector)
            .andThen(getAndScoreAlgaeFromCoral(scoreAlgae1, pullAway1, elevatorArmIntakeHandler, compositions, endEffector),
                getAndScoreAlgaeFromBarge(getAlgae2, scoreAlgae2, elevatorArmIntakeHandler, compositions, endEffector));
    }

    private static Command getAndScoreAlgaeFromCoral(PathPlannerPath getAndScoreAlgaePath, PathPlannerPath pullAwayPath, ElevatorArmIntakeHandler elevatorArmIntakeHandler, Compositions compositions, EndEffector endEffector) {
        return 
            /* Follow get algae and score algae paths */
            Commands.sequence(
                elevatorArmIntakeHandler.moveToStowPositions(),
                AutoBuilder.followPath(pullAwayPath),
                elevatorArmIntakeHandler.prepareForAlgaeReefIntakingAuto(),
                AutoBuilder.followPath(getAndScoreAlgaePath)
                    .alongWith(endEffector.pickupAlgae().asProxy()),
                /* Score the algae when the paths finish and everything is at setpoint */
                elevatorArmIntakeHandler.prepareForAlgaeScoringAuto(),
                endEffector.outtakeNetAlgae().asProxy()
            );
    }

    private static Command getAndScoreAlgaeFromBarge(PathPlannerPath getPath, PathPlannerPath scorePath, ElevatorArmIntakeHandler elevatorArmIntakeHandler, Compositions compositions, EndEffector endEffector) {
        return 
            /* Follow get algae and score algae paths */
            Commands.sequence(
                elevatorArmIntakeHandler.moveToStowPositions(),
                AutoBuilder.followPath(getPath)
                    .alongWith(
                        elevatorArmIntakeHandler.prepareForAlgaeReefIntakingAuto(),
                        endEffector.pickupAlgae().asProxy()
                    ),
                AutoBuilder.followPath(scorePath),
                /* Score the algae when the paths finish and everything is at setpoint */
                elevatorArmIntakeHandler.prepareForAlgaeScoringAuto(),
                endEffector.outtakeNetAlgae().asProxy()
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
                endEffector.placeCoral().asProxy(),
                Commands.runOnce(() -> shouldMoveElevator = false)
            );
    }

    private static Command scorePreloadCoral(PathPlannerPath scoreCoralPath, ElevatorArmIntakeHandler elevatorArmIntakeHandler, EndEffector endEffector) {
        return Commands.sequence(
            AutoBuilder.followPath(scoreCoralPath)
                .alongWith(elevatorArmIntakeHandler.prepareForAutoCoralScoring(() -> shouldMoveElevator))
                    .deadlineFor(endEffector.holdCoral().asProxy()),
            Commands.waitSeconds(0.25),//TODO d
            endEffector.placeCoral().asProxy(),
            Commands.runOnce(() -> shouldMoveElevator = false)
        );
    }
}
