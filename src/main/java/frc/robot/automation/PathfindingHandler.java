package frc.robot.automation;

import static frc.robot.Constants.PathplannerConfig.kDynamicPathConstraints;

import java.util.Map;
import java.util.function.Supplier;

import static frc.robot.Constants.ObjectAimingConstants.kIntakeOffset;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.automation.ButtonBoardHandler.ReefPath;
import frc.robot.commands.DriveToMovingPose;
import frc.robot.localization.TrackedObject.ObjectType;
import frc.robot.subsystems.swerve.KrakenSwerve;
import frc.robot.utils.ElasticUtil;

public class PathfindingHandler {
    private PathfindingHandler() {}

    private static Command pathTo(String pathName) {
        try {
            PathPlannerPath pathToFollow = PathPlannerPath.fromPathFile(pathName);

            return AutoBuilder.pathfindThenFollowPath(pathToFollow, kDynamicPathConstraints);
        } catch (Exception e) {
            ElasticUtil.sendError("Path " + pathName + " failed to load!", "Automatic cycling will not work");

            return Commands.none();
        }
    }

    // private static Command pathToReef(KrakenSwerve swerve, String pathName) {
    //     try {
    //         PathPlannerPath pathToFollow = PathPlannerPath.fromPathFile(pathName);

    //         if (AutoBuilder.shouldFlip())
    //             pathToFollow = pathToFollow.flipPath();

    //         Translation2d endTrans = pathToFollow.getWaypoints().get(1).anchor();

    //         return new AlignWithPose(swerve, new Pose2d(endTrans, pathToFollow.getGoalEndState().rotation()));
    //     } catch (Exception e) {
    //         ElasticUtil.sendError("Path " + pathName + " failed to load!", "Automatic cycling will not work");

    //         return Commands.none();
    //     }
    // }

    private static Pose2d getCoralAlignmentPose(KrakenSwerve swerve) {
        var optionalCoral = swerve.getLocalization().getNearestObjectOfType(ObjectType.CORAL);

        if (optionalCoral.isPresent()) {
            var robotTranslation = swerve.getLocalization().getCurrentPose().getTranslation();
            var coralTranslation = optionalCoral.get().getFieldPosition().getTranslation();

            /* Find the angle that points the robot towards the coral */
            var rotationSetpoint = robotTranslation.minus(coralTranslation).getAngle();

            var targetPose = new Pose2d(coralTranslation, rotationSetpoint)
                /* Apply the intake's offset  */
                .transformBy(kIntakeOffset);

            return targetPose;
        }

        /* If no coral is seen, maintain the current pose */
        else    
            return swerve.getLocalization().getCurrentPose();
    }

    public static Command pathToNearestCoral(KrakenSwerve swerve) {
        return new DriveToMovingPose(swerve, () -> getCoralAlignmentPose(swerve));
    }

    public static Command pathToLSource() {
        return pathTo("LSource");
    }

    public static Command pathToBarge() {
        return pathTo("Barge");
    }

    public static Command pathToRSource() {
        return pathTo("RSource");
    }

    public static Command pathToProcessor() {
        return pathTo("Processor");
    }

    public static Command pathToReefSide(Supplier<ReefPath> reefPathSup) {
        return new SelectCommand<ReefPath>(
            Map.ofEntries(
                Map.entry(ReefPath.NONE, Commands.none()),
                Map.entry(ReefPath.TO_1L, pathTo("1L")),
                Map.entry(ReefPath.TO_1R, pathTo("1R")),
                Map.entry(ReefPath.TO_2L, pathTo("2L")),
                Map.entry(ReefPath.TO_2R, pathTo("2R")),
                Map.entry(ReefPath.TO_3L, pathTo("3L")),
                Map.entry(ReefPath.TO_3R, pathTo("3R")),
                Map.entry(ReefPath.TO_4L, pathTo("4L")),
                Map.entry(ReefPath.TO_4R, pathTo("4R")),
                Map.entry(ReefPath.TO_5L, pathTo("5L")),
                Map.entry(ReefPath.TO_5R, pathTo("5R")),
                Map.entry(ReefPath.TO_6L, pathTo("6L")),
                Map.entry(ReefPath.TO_6R, pathTo("6R"))
            ),
            reefPathSup
        );
    }

    public static Command pathToAlgaeIntakeFromReef(Supplier<ReefPath> reefPathSup) {
        return new SelectCommand<ReefPath>(
            Map.ofEntries(
                Map.entry(ReefPath.NONE, Commands.none()),
                Map.entry(ReefPath.TO_1L, pathTo("1A")),
                Map.entry(ReefPath.TO_1R, pathTo("1A")),
                Map.entry(ReefPath.TO_2L, pathTo("2A")),
                Map.entry(ReefPath.TO_2R, pathTo("2A")),
                Map.entry(ReefPath.TO_3L, pathTo("3A")),
                Map.entry(ReefPath.TO_3R, pathTo("3A")),
                Map.entry(ReefPath.TO_4L, pathTo("4A")),
                Map.entry(ReefPath.TO_4R, pathTo("4A")),
                Map.entry(ReefPath.TO_5L, pathTo("5A")),
                Map.entry(ReefPath.TO_5R, pathTo("5A")),
                Map.entry(ReefPath.TO_6L, pathTo("6A")),
                Map.entry(ReefPath.TO_6R, pathTo("6A"))
            ),
            reefPathSup
        );
    }
}
