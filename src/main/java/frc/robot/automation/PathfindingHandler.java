package frc.robot.automation;

import static frc.robot.Constants.PathFollowingConstants.*;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutopilotDriveToPose;
import frc.robot.commands.DriveToMovingPose;
import frc.robot.commands.RotationalDriveToCoral;
import frc.robot.localization.TrackedObject.ObjectType;
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

            return AutoBuilder.pathfindThenFollowPath(pathToFollow, kDynamicPathConstraints);
        } catch (Exception e) {
            ElasticUtil.sendError("Path " + pathName + " failed to load!", "Autos will not work");

            return Commands.none();
        }
    }

    private static Command pathToPoseWithIntermediate(Pose2d endPose, Pose2d startPose) {
        var intermediatePose = endPose.transformBy(kIntermediatePointOffset);

        var directionToIntermediate = startPose.getTranslation().minus(intermediatePose.getTranslation()).getAngle().plus(Rotation2d.k180deg);;
        var directionToFinal = intermediatePose.getTranslation().minus(endPose.getTranslation()).getAngle().plus(Rotation2d.k180deg);;

        var waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(startPose.getTranslation(), directionToIntermediate),
            /* Add an intermediate waypoint to make sure we path in straight */
            new Pose2d(intermediatePose.getTranslation(), directionToFinal),
            new Pose2d(endPose.getTranslation(), directionToFinal)
        );

        var path = new PathPlannerPath(
            waypoints, 
            kDynamicPathConstraints, 
            null, 
            new GoalEndState(0, endPose.getRotation())
        );
        path.preventFlipping = true;

        return AutoBuilder.followPath(path);
    }

    private static Command pathToPose(Pose2d endPose, Pose2d startPose) {
        var directionToFinal = startPose.getTranslation().minus(endPose.getTranslation()).getAngle().plus(Rotation2d.k180deg);;

        var waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(startPose.getTranslation(), directionToFinal),
            new Pose2d(endPose.getTranslation(), directionToFinal)
        );

        var path = new PathPlannerPath(
            waypoints, 
            kDynamicPathConstraints, 
            null, 
            new GoalEndState(0, endPose.getRotation())
        );
        path.preventFlipping = true;

        return AutoBuilder.followPath(path);
    }

    private static Pose2d getCoralAlignmentPose(KrakenSwerve swerve) {
        var optionalCoral = swerve.getLocalization().getNearestObjectOfType(ObjectType.CORAL);

        if (optionalCoral.isPresent()) {
            var robotTranslation = swerve.getLocalization().getCurrentPose().getTranslation();
            var coralTranslation = optionalCoral.get().getFieldPosition().getTranslation();

            /* Find the angle that points the robot towards the coral */
            var rotationSetpoint = robotTranslation.minus(coralTranslation).getAngle()//TODO this causes stability problems while driving if done in real time
                .plus(Rotation2d.k180deg);

            var targetPose = new Pose2d(coralTranslation, rotationSetpoint)
                /* Apply the intake's offset  */
                .transformBy(kCoralIntakeOffset);

            return targetPose;
        }

        /* If no coral is seen, maintain the current pose */
        else    
            return swerve.getLocalization().getCurrentPose();
    }

    public static Command pathToNearestCoral(KrakenSwerve swerve) {
        return AutopilotDriveToPose.enterAtTargetRotation(swerve, () -> getCoralAlignmentPose(swerve));
    }

    public static Command pathToNearestMovingCoral(KrakenSwerve swerve) {//TODO this method is broken
        return new DriveToMovingPose(swerve, () -> getCoralAlignmentPose(swerve));
    }

    public static Command pathToBarge(KrakenSwerve swerve) {
        return Commands.defer(
            () -> pathToPose(
                FieldUtil.getBargeAlignmentPose(swerve.getLocalization()), 
                swerve.getLocalization().getCurrentPose()
            ), 
            Set.of(swerve)
        );
    }

    public static Command pathToClosestReefBranch(KrakenSwerve swerve, boolean isLeftBranch) {
        return Commands.defer(
            () -> pathToPoseWithIntermediate(
                FieldUtil.getClosestReefScoringLocation(swerve.getLocalization(), isLeftBranch), 
                swerve.getLocalization().getCurrentPose()
            ), 
            Set.of(swerve)
        );
    }

    public static Command pathToReefBranchDynamic(int reefTagID, KrakenSwerve swerve, boolean isLeftBranch) {
        return Commands.defer(
            () -> pathToPoseWithIntermediate(
                FieldUtil.getReefScoringLocationFromTagID(swerve.getLocalization(), isLeftBranch, reefTagID), 
                swerve.getLocalization().getCurrentPose()
            ), 
            Set.of(swerve)
        );
    }

    public static Command pathToReefBranch(int reefTagID, KrakenSwerve swerve, boolean isLeftBranch, Pose2d startPose) {
        return pathToPoseWithIntermediate(FieldUtil.getReefScoringLocationFromTagID(swerve.getLocalization(), isLeftBranch, reefTagID), startPose);
    }
    
    public static Command pathToClosestReefAlgaeIntake(KrakenSwerve swerve) {
        return AutopilotDriveToPose.enterAtTargetRotation(swerve, () -> FieldUtil.getClosestReefAlgaeIntakeLocation(swerve.getLocalization()));
    }

    public static Command pathToSourceThenCoral(KrakenSwerve swerve, boolean isLeftSource) {
        return pathToStubPath("LSourceToCoral", !isLeftSource)
            .andThen(new RotationalDriveToCoral(swerve));
    }
}
