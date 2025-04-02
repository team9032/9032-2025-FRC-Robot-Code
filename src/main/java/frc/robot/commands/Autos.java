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
    private static final Trigger prepareForAutoScoring = new EventTrigger("ElevatorAuto");

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
            AutoBuilder.followPath(score1)
                .deadlineFor(endEffector.holdCoral().asProxy())
                    .alongWith(prepareForScore(elevatorArmIntakeHandler)),
            Commands.waitSeconds(0.5),//TODO no.
            endEffector.placeCoral().asProxy(),
            /* Get coral 2 */
            AutoBuilder.followPath(get2)
                .alongWith(intake(elevatorArmIntakeHandler, compositions)),
            /* Score coral 2 */
            AutoBuilder.followPath(score2)
                .deadlineFor(endEffector.holdCoral().asProxy())
                    .alongWith(prepareForScore(elevatorArmIntakeHandler)),
            Commands.waitSeconds(0.5),//TODO no.
            endEffector.placeCoral().asProxy(),
            /* Get coral 3 */
            AutoBuilder.followPath(get3)
                .alongWith(intake(elevatorArmIntakeHandler, compositions)),
            /* Score coral 3 */
            AutoBuilder.followPath(score3)
                .deadlineFor(endEffector.holdCoral().asProxy())
                    .alongWith(prepareForScore(elevatorArmIntakeHandler)),
            endEffector.placeCoral().asProxy(),
            /* Get coral 4 */
            AutoBuilder.followPath(get4)
                .alongWith(intake(elevatorArmIntakeHandler, compositions)),
            /* Score coral 4 */
            AutoBuilder.followPath(score4)
                .deadlineFor(endEffector.holdCoral().asProxy())
                    .alongWith(prepareForScore(elevatorArmIntakeHandler)),
            endEffector.placeCoral().asProxy(),
            /* Return to stow positions */
            elevatorArmIntakeHandler.moveToStowPositions()
        );
    }

    // public static Command oneCoralCenter(Elevator elevator, Arm arm, EndEffector endEffector, KrakenSwerve swerve, Intake intake, Indexer indexer) {
    //     try {
    //         return Commands.sequence(
    //             /* Score preload */
    //             AutoBuilder.followPath(PathPlannerPath.fromPathFile("Center 1C1"))
    //                 .deadlineFor(endEffector.holdCoral())
    //                     .alongWith(prepareForScore(elevator, arm)),
    //             endEffector.placeCoral(),
    //             /* Return to stow positions */
    //             arm.moveToStowPos(),
    //             Commands.waitUntil(arm::atSetpoint),
    //             elevator.moveToIndexerPosition()
    //         );
    //     } catch (Exception e) {
    //         ElasticUtil.sendError("Could not load auto path!", "Auto will not work!");

    //         return Commands.none();
    //     }
    // }

    private static Command prepareForScore(ElevatorArmIntakeHandler elevatorArmIntakeHandler) {
        return Commands.sequence(
            elevatorArmIntakeHandler.moveToStowPositions(),
            Commands.waitUntil(prepareForAutoScoring),//TODO this is a race condition
            elevatorArmIntakeHandler.prepareForAutoCoralScoring(),
            ElasticUtil.sendInfoCommand("Prepared for coral scoring in auto")
        );
    }

    private static Command intake(ElevatorArmIntakeHandler elevatorArmIntakeHandler, Compositions compositions) {
        return Commands.sequence(
            elevatorArmIntakeHandler.moveToIntakePosition(),
            compositions.intakeCoralToEndEffector(false)
        );
    }
}
