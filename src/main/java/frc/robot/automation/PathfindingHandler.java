package frc.robot.automation;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SimpleDriveToPose;
import frc.robot.pathing.CurvedPath;
import frc.robot.commands.FollowCurvedPath;
import frc.robot.subsystems.swerve.KrakenSwerve;
import frc.robot.utils.FieldUtil;

public class PathfindingHandler {
    private PathfindingHandler() {}

    private static Command pathToPose(Supplier<Pose2d> endPoseSup, KrakenSwerve swerve) {
        return Commands.defer(
            () -> new FollowCurvedPath(
                swerve, 
                CurvedPath.createStraightPath(swerve.getLocalization().getCurrentPose().getTranslation(), endPoseSup.get())
            ),
            Set.of(swerve)
        );
    }

    private static Command pathToPoseWithCurvature(Supplier<Pose2d> endPoseSup, KrakenSwerve swerve) {
        return Commands.defer(
            () -> new FollowCurvedPath(
                swerve, 
                CurvedPath.enterAtFinalRotation(endPoseSup.get(), true)
            ),
            Set.of(swerve)
        );
    }
    
    public static Command pathToBarge(KrakenSwerve swerve) {
        return pathToPose(() -> FieldUtil.getBargeAlignmentPose(swerve.getLocalization()), swerve);
    }

    public static Command pathToClosestReefBranch(KrakenSwerve swerve, boolean isLeftBranch) {
        return pathToPoseWithCurvature(() -> FieldUtil.getClosestReefScoringLocation(swerve.getLocalization(), isLeftBranch), swerve);
    }

    public static Command pathToClosestOffsetReefBranch(KrakenSwerve swerve, boolean isLeftBranch) {
        return pathToPoseWithCurvature(() -> FieldUtil.getClosestOffsetReefScoringLocation(swerve.getLocalization(), isLeftBranch), swerve);
    }

    public static Command pathToReefBranch(int reefTagID, KrakenSwerve swerve, boolean isLeftBranch) {
        return pathToPoseWithCurvature(() -> FieldUtil.getReefScoringLocationFromTagID(swerve.getLocalization(), isLeftBranch, reefTagID), swerve);
    }

    public static Command pathToOffsetReefBranch(int reefTagID, KrakenSwerve swerve, boolean isLeftBranch) {
        return pathToPoseWithCurvature(() -> FieldUtil.getOffsetReefScoringLocationFromTagID(swerve.getLocalization(), isLeftBranch, reefTagID), swerve);
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
    
    public static Command pathToClosestReefAlgaeIntake(KrakenSwerve swerve) {
        return pathToPoseWithCurvature(() -> FieldUtil.getClosestReefAlgaeIntakeLocation(swerve.getLocalization()), swerve);
    }
}
