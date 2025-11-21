package frc.robot.automation;

import static frc.robot.Constants.PathFollowingConstants.*;

import java.util.Set;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SimpleDriveToPose;
import frc.robot.commands.RotationalDriveToCoral;
import frc.robot.subsystems.swerve.KrakenSwerve;
import frc.robot.utils.ElasticUtil;
import frc.robot.utils.FieldUtil;

public class PathfindingHandler {
    private PathfindingHandler() {}

    private static Command pathToStubPath(String pathName, boolean mirror) {
        try {
            PathPlannerPath pathToFollow = PathPlannerPath.fromPathFile(pathName);
            
            if (mirror)
                pathToFollow = pathToFollow.mirrorPath();

            return AutoBuilder.pathfindThenFollowPath(pathToFollow, kNormalPathConstraints);
        } catch (Exception e) {
            ElasticUtil.sendError("Path " + pathName + " failed to load!", "Autos will not work");

            return Commands.none();
        }
    }

    private static PathPlannerPath getPathWithIntermediate(Pose2d endPose, Pose2d startPose, boolean slow) {
        /* Don't use intermediate waypoint when close */
        if (startPose.getTranslation().getDistance(endPose.getTranslation()) < kIntermediateStartDistance)
            return getPath(endPose, startPose);

        var intermediatePose = endPose.transformBy(kIntermediatePointOffset);

        var directionToIntermediate = startPose.getTranslation().minus(intermediatePose.getTranslation()).getAngle().plus(Rotation2d.k180deg);
        var directionToFinal = intermediatePose.getTranslation().minus(endPose.getTranslation()).getAngle().plus(Rotation2d.k180deg);

        var waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(startPose.getTranslation(), directionToIntermediate),
            /* Add an intermediate waypoint to make sure we path in straight */
            new Pose2d(intermediatePose.getTranslation(), directionToFinal),
            new Pose2d(endPose.getTranslation(), directionToFinal)
        );

        var path = new PathPlannerPath(
            waypoints, 
            slow ? kSlowPathConstraints : kNormalPathConstraints, 
            null, 
            new GoalEndState(0, endPose.getRotation())
        );
        path.preventFlipping = true;

        return path;
    }

    private static Command pathToPoseWithIntermediate(Supplier<Pose2d> endPoseSup, KrakenSwerve swerve, boolean slow) {
        return Commands.defer(
            () -> AutoBuilder.followPath(
                getPathWithIntermediate(
                    endPoseSup.get(),
                    swerve.getLocalization().getCurrentPose(),
                    slow
                )
            ),
            Set.of(swerve)
        );
    }

    private static PathPlannerPath getPath(Pose2d endPose, Pose2d startPose) {
        var directionToFinal = startPose.getTranslation().minus(endPose.getTranslation()).getAngle().plus(Rotation2d.k180deg);

        var waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(startPose.getTranslation(), directionToFinal),
            new Pose2d(endPose.getTranslation(), directionToFinal)
        );

        var path = new PathPlannerPath(
            waypoints, 
            kNormalPathConstraints, 
            null, 
            new GoalEndState(0, endPose.getRotation())
        );
        path.preventFlipping = true;

        return path;
    }

    private static Command pathToPose(Supplier<Pose2d> endPoseSup, KrakenSwerve swerve) {
        return Commands.defer(
            () -> AutoBuilder.followPath(
                getPath(
                    endPoseSup.get(),
                    swerve.getLocalization().getCurrentPose()
                )
            ),
            Set.of(swerve)
        );
    }

    public static Command pathToBarge(KrakenSwerve swerve) {
        return pathToPose(() -> FieldUtil.getBargeAlignmentPose(swerve.getLocalization()), swerve);
    }

    public static Command pathToClosestReefBranch(KrakenSwerve swerve, boolean isLeftBranch) {
        return pathToPoseWithIntermediate(() -> FieldUtil.getClosestReefScoringLocation(swerve.getLocalization(), isLeftBranch), swerve, true);
    }

    public static Command pathToClosestOffsetReefBranch(KrakenSwerve swerve, boolean isLeftBranch) {
        return pathToPoseWithIntermediate(() -> FieldUtil.getClosestOffsetReefScoringLocation(swerve.getLocalization(), isLeftBranch), swerve, true);
    }

    public static Command pathToReefBranch(int reefTagID, KrakenSwerve swerve, boolean isLeftBranch) {
        return pathToPoseWithIntermediate(() -> FieldUtil.getReefScoringLocationFromTagID(swerve.getLocalization(), isLeftBranch, reefTagID), swerve, true);
    }

    public static Command pathToOffsetReefBranch(int reefTagID, KrakenSwerve swerve, boolean isLeftBranch) {
        return pathToPoseWithIntermediate(() -> FieldUtil.getOffsetReefScoringLocationFromTagID(swerve.getLocalization(), isLeftBranch, reefTagID), swerve, false);
    }

    public static Command simpleDriveToReefBranch(int reefTagID, KrakenSwerve swerve, boolean isLeftBranch) {
        return Commands.defer(
            () -> new SimpleDriveToPose(swerve, FieldUtil.getReefScoringLocationFromTagID(swerve.getLocalization(), isLeftBranch, reefTagID)),
            Set.of(swerve)
        );
    }
    public static Command simpleDriveClosestToReefBranch(KrakenSwerve swerve, boolean isLeftBranch) {
        return Commands.defer(
            () -> new SimpleDriveToPose(swerve, FieldUtil.getClosestReefScoringLocation(swerve.getLocalization(), isLeftBranch)),
            Set.of(swerve)
        );
    }

    public static PathPlannerPath getReefBranchPath(int reefTagID, KrakenSwerve swerve, boolean isLeftBranch, Pose2d startPose) {
        return getPath(FieldUtil.getReefScoringLocationFromTagID(swerve.getLocalization(), isLeftBranch, reefTagID), startPose);
    }
    
    public static Command pathToClosestReefAlgaeIntake(KrakenSwerve swerve) {
        return pathToPoseWithIntermediate(() -> FieldUtil.getClosestReefAlgaeIntakeLocation(swerve.getLocalization()), swerve, false);
    }

    public static Command pathToSourceThenCoral(KrakenSwerve swerve, boolean isLeftSource) {
        return pathToStubPath("LSourceToCoral", !isLeftSource)
            .andThen(new RotationalDriveToCoral(swerve));
    }
}
