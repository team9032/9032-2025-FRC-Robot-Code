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
        PathPlannerPath path1;
        PathPlannerPath path2;
        PathPlannerPath path3;
        PathPlannerPath path4;
        PathPlannerPath path5;
        PathPlannerPath path6;
        PathPlannerPath path7;

        try {
            path1 = PathPlannerPath.fromPathFile("Score Coral 1");
            path2 = PathPlannerPath.fromPathFile("Get Coral 2");
            path3 = PathPlannerPath.fromPathFile("Score Coral 2");
            path4 = PathPlannerPath.fromPathFile("Get Coral 3");
            path5 = PathPlannerPath.fromPathFile("Score Coral 3");
            path6 = PathPlannerPath.fromPathFile("Get Coral 4");
            path7 = PathPlannerPath.fromPathFile("Score Coral 4");

            if (mirrored) {
                path1 = path1.mirrorPath();
                path2 = path2.mirrorPath();
                path3 = path3.mirrorPath();
                path4 = path4.mirrorPath();
                path5 = path5.mirrorPath();
                path6 = path6.mirrorPath();
                path7 = path7.mirrorPath();
            }
        } catch (Exception e) {
            ElasticUtil.sendError("Could not load auto path!", "Auto will not work!");

            return Commands.none();
        }

        return Commands.sequence(
            /* Score preload */
            AutoBuilder.followPath(path1)
                .deadlineFor(endEffector.holdCoral().asProxy())
                    .alongWith(prepareForScore(elevatorArmIntakeHandler)),
            Commands.waitSeconds(0.5),//TODO no.
            endEffector.placeCoral().asProxy(),
            /* Get coral 2 */
            AutoBuilder.followPath(path2)
                .alongWith(intake(elevatorArmIntakeHandler, compositions)),
            /* Score coral 2 */
            AutoBuilder.followPath(path3)
                .deadlineFor(endEffector.holdCoral().asProxy())
                    .alongWith(prepareForScore(elevatorArmIntakeHandler)),
            Commands.waitSeconds(0.5),//TODO no.
            endEffector.placeCoral().asProxy(),
            /* Get coral 3 */
            AutoBuilder.followPath(path4)
                .alongWith(intake(elevatorArmIntakeHandler, compositions)),
            /* Score coral 3 */
            AutoBuilder.followPath(path5)
                .deadlineFor(endEffector.holdCoral().asProxy())
                    .alongWith(prepareForScore(elevatorArmIntakeHandler)),
            endEffector.placeCoral().asProxy(),
            /* Get coral 4 */
            AutoBuilder.followPath(path6)
                .alongWith(intake(elevatorArmIntakeHandler, compositions)),
            /* Score coral 4 */
            AutoBuilder.followPath(path7)
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
