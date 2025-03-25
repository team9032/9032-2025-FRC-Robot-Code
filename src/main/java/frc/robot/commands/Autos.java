package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.KrakenSwerve;
import frc.robot.utils.ElasticUtil;

public class Autos {
    private static final Trigger prepareForAutoScoring = new EventTrigger("ElevatorAuto");

    public static Command fourCoralLeft(Elevator elevator, Arm arm, EndEffector endEffector, KrakenSwerve swerve, Intake intake, Indexer indexer, Boolean mirrored) {
        PathPlannerPath path1;
        PathPlannerPath path2;
        PathPlannerPath path3;
        PathPlannerPath path4;
        PathPlannerPath path5;
        PathPlannerPath path6;
        PathPlannerPath path7;

        try {
            path1 = PathPlannerPath.fromPathFile("Left 3C1");
            path2 = PathPlannerPath.fromPathFile("Left 3C2");
            path3 = PathPlannerPath.fromPathFile("Left 3C3");
            path4 = PathPlannerPath.fromPathFile("Left 3C4");
            path5 = PathPlannerPath.fromPathFile("Left 3C5");
            path6 = PathPlannerPath.fromPathFile("Left 4C1");
            path7 = PathPlannerPath.fromPathFile("Left 4C2");

            if (mirrored) {
                path1 = path1.mirrorPath();
                path2 = path1.mirrorPath();
                path3 = path1.mirrorPath();
                path4 = path1.mirrorPath();
                path5 = path1.mirrorPath();
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
                .deadlineFor(endEffector.holdCoral())
                    .alongWith(prepareForScore(elevator, arm)),
            endEffector.placeCoral(),
            /* Get coral 2 */
            AutoBuilder.followPath(path2)
                .alongWith(returnToStowAndPrepareForIntake(intake, arm, elevator)),
            new AimAtCoral(swerve, intake::getObstacleSensorDistance, true)
                .until(endEffector::hasCoral)
                    .alongWith(intakeToEndEffector(intake, indexer, endEffector, arm)),
            /* Score coral 2 */
            AutoBuilder.followPath(path3)
                    .deadlineFor(endEffector.holdCoral())
                        .alongWith(prepareForScore(elevator, arm)),
            endEffector.placeCoral(),
            /* Get coral 3 */
            AutoBuilder.followPath(path4)
                .alongWith(returnToStowAndPrepareForIntake(intake, arm, elevator)),
            new AimAtCoral(swerve, intake::getObstacleSensorDistance, true)
                .until(endEffector::hasCoral)
                    .alongWith(intakeToEndEffector(intake, indexer, endEffector, arm)),
            /* Score coral 3 */
            AutoBuilder.followPath(path5)
                    .deadlineFor(endEffector.holdCoral())
                        .alongWith(prepareForScore(elevator, arm)),
            endEffector.placeCoral(),
            /* Get coral 4 */
            AutoBuilder.followPath(path6)
                    .alongWith(returnToStowAndPrepareForIntake(intake, arm, elevator)),
                new AimAtCoral(swerve, intake::getObstacleSensorDistance, true)
                    .until(endEffector::hasCoral)
                        .alongWith(intakeToEndEffector(intake, indexer, endEffector, arm)),
                /* Score coral 4 */
                AutoBuilder.followPath(path7)
                        .deadlineFor(endEffector.holdCoral())
                            .alongWith(prepareForScore(elevator, arm)),
                endEffector.placeCoral(),
            /* Return to stow positions */
            arm.moveToStowPos(),
            Commands.waitUntil(arm::atSetpoint),
            elevator.moveToIndexerPosition()
        );
    }

    public static Command threeCoralLeft(Elevator elevator, Arm arm, EndEffector endEffector, KrakenSwerve swerve, Intake intake, Indexer indexer, Boolean mirrored) {
        PathPlannerPath path1;
        PathPlannerPath path2;
        PathPlannerPath path3;
        PathPlannerPath path4;
        PathPlannerPath path5;

        try {
            path1 = PathPlannerPath.fromPathFile("Left 3C1");
            path2 = PathPlannerPath.fromPathFile("Left 3C2");
            path3 = PathPlannerPath.fromPathFile("Left 3C3");
            path4 = PathPlannerPath.fromPathFile("Left 3C4");
            path5 = PathPlannerPath.fromPathFile("Left 3C5");

            if (mirrored) {
                path1 = path1.mirrorPath();
                path2 = path1.mirrorPath();
                path3 = path1.mirrorPath();
                path4 = path1.mirrorPath();
                path5 = path1.mirrorPath();
            }
        } catch (Exception e) {
            ElasticUtil.sendError("Could not load auto path!", "Auto will not work!");

            return Commands.none();
        }

        return Commands.sequence(
            /* Score preload */
            AutoBuilder.followPath(path1)
                .deadlineFor(endEffector.holdCoral())
                    .alongWith(prepareForScore(elevator, arm)),
            endEffector.placeCoral(),
            /* Get coral 2 */
            AutoBuilder.followPath(path2)
                .alongWith(returnToStowAndPrepareForIntake(intake, arm, elevator)),
            new AimAtCoral(swerve, intake::getObstacleSensorDistance, true)
                .until(endEffector::hasCoral)
                    .alongWith(intakeToEndEffector(intake, indexer, endEffector, arm)),
            /* Score coral 2 */
            AutoBuilder.followPath(path3)
                    .deadlineFor(endEffector.holdCoral())
                        .alongWith(prepareForScore(elevator, arm)),
            endEffector.placeCoral(),
            /* Get coral 3 */
            AutoBuilder.followPath(path4)
                .alongWith(returnToStowAndPrepareForIntake(intake, arm, elevator)),
            new AimAtCoral(swerve, intake::getObstacleSensorDistance, true)
                .until(endEffector::hasCoral)
                    .alongWith(intakeToEndEffector(intake, indexer, endEffector, arm)),
            /* Score coral 3 */
            AutoBuilder.followPath(path5)
                    .deadlineFor(endEffector.holdCoral())
                        .alongWith(prepareForScore(elevator, arm)),
            endEffector.placeCoral(),
            /* Return to stow positions */
            arm.moveToStowPos(),
            Commands.waitUntil(arm::atSetpoint),
            elevator.moveToIndexerPosition()
        );
    }

    public static Command oneCoralCenter(Elevator elevator, Arm arm, EndEffector endEffector, KrakenSwerve swerve, Intake intake, Indexer indexer) {
        try {
            return Commands.sequence(
                /* Score preload */
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("Center 1C1"))
                    .deadlineFor(endEffector.holdCoral())
                        .alongWith(prepareForScore(elevator, arm)),
                endEffector.placeCoral(),
                /* Return to stow positions */
                arm.moveToStowPos(),
                Commands.waitUntil(arm::atSetpoint),
                elevator.moveToIndexerPosition()
            );
        } catch (Exception e) {
            ElasticUtil.sendError("Could not load auto path!", "Auto will not work!");

            return Commands.none();
        }
    }

    private static Command prepareForScore(Elevator elevator, Arm arm) {
        return Commands.sequence(
            Commands.waitUntil(prepareForAutoScoring),
            elevator.moveToL3Position(),
            Commands.waitUntil(elevator::atSetpoint),
            arm.moveToLevel3Pos(),
            Commands.waitUntil(arm::atSetpoint),
            ElasticUtil.sendInfoCommand("Prepared for coral scoring in auto")
        );
    }

    private static Command returnToStowAndPrepareForIntake(Intake intake, Arm arm, Elevator elevator) {
        return Commands.sequence(
            elevator.moveToIndexerPosition(),
            arm.moveToStowPos(),
            Commands.waitUntil(elevator::atSetpoint),
            arm.moveToIndexerPos(),
            intake.moveToGround()
        );
    }

    private static Command intakeToEndEffector(Intake intake, Indexer indexer, EndEffector endEffector, Arm arm) {
        return Commands.sequence(
            Commands.waitUntil(intake::canRunRollers),
            intake.intakeCoral(),
            indexer.spinRollers(),
            endEffector.receiveCoralFromIndexer(),
            intake.stopIntaking(),
            indexer.stopRollers(),
            intake.returnToStowPosition(),
            arm.moveToStowPos()
        );
    }
}
