package frc.robot.automation;

import static frc.robot.Constants.PathplannerConfig.kDynamicPathConstraints;
import static frc.robot.Constants.ObjectAimingConstants.kIntakeOffset;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public static Command pathTo1L(KrakenSwerve swerve) {
        return pathTo("1L");
    }

    public static Command pathTo1R(KrakenSwerve swerve) {
        return pathTo("1R");
    }

    public static Command pathTo2L(KrakenSwerve swerve) {
        return pathTo("2L");
    }

    public static Command pathTo2R(KrakenSwerve swerve) {
        return pathTo("2R");
    }

    public static Command pathTo3L(KrakenSwerve swerve) {
        return pathTo("3L");
    }

    public static Command pathTo3R(KrakenSwerve swerve) {
        return pathTo( "3R");
    }

    public static Command pathTo4L(KrakenSwerve swerve) {
        return pathTo("4L");
    }

    public static Command pathTo4R(KrakenSwerve swerve) {
        return pathTo("4R");
    }

    public static Command pathTo5L(KrakenSwerve swerve) {
        return pathTo("5L");
    }

    public static Command pathTo5R(KrakenSwerve swerve) {
        return pathTo("5R");
    }

    public static Command pathTo6L(KrakenSwerve swerve) {
        return pathTo("6L");
    }
    
    public static Command pathTo6R(KrakenSwerve swerve) {
        return pathTo("6R");
    }

    public static Command pathTo1A(KrakenSwerve swerve) {
        return pathTo("1A");
    }

    public static Command pathTo2A(KrakenSwerve swerve) {
        return pathTo("2A");
    }

    public static Command pathTo3A(KrakenSwerve swerve) {
        return pathTo("3A");
    }

    public static Command pathTo4A(KrakenSwerve swerve) {
        return pathTo("4A");
    }

    public static Command pathTo5A(KrakenSwerve swerve) {
        return pathTo("5A");
    }

    public static Command pathTo6A(KrakenSwerve swerve) {
        return pathTo("6A");
    } 
}
