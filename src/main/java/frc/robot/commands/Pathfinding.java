package frc.robot.commands;

import static frc.robot.Constants.PathplannerConfig.kDynamicPathConstraints;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.KrakenSwerve;
import frc.robot.utils.ElasticUtil;

public class Pathfinding {
    private Pathfinding() {}

    private static Command pathTo(KrakenSwerve swerve, String pathName) {
        return Commands.defer(() -> getFollowingAndAlignmentCommand(swerve, pathName), Set.of(swerve));
    }

    private static Command getFollowingAndAlignmentCommand(KrakenSwerve swerve, String pathName) {
        try {
            PathPlannerPath pathToFollow = PathPlannerPath.fromPathFile(pathName);
            if(DriverStation.getAlliance().get().equals(Alliance.Red))//Maintain blue origin
                pathToFollow = pathToFollow.flipPath();

            Translation2d endTrans = pathToFollow.getWaypoints().get(1).anchor();//Path endpoints should be at the reef

            var endingDriveDirection = pathToFollow.getGoalEndState().rotation().plus(Rotation2d.k180deg);

            var waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(swerve.drivetrain.getState().Pose.getTranslation(), endingDriveDirection),
                new Pose2d(endTrans, endingDriveDirection)
            );

            var path = new PathPlannerPath(
                waypoints, 
                kDynamicPathConstraints, 
                null, 
                pathToFollow.getGoalEndState()
            );

            path.preventFlipping = true;
                
            var endPose = new Pose2d(endTrans, pathToFollow.getGoalEndState().rotation());

            return AutoBuilder.followPath(path)
                .andThen(new AlignWithPose(swerve, endPose))
                .onlyIf(() -> !swerve.drivetrain.getState().Pose.equals(endPose));
        } catch (Exception e) {
            ElasticUtil.sendError("Path " + pathName + " failed to load!", "Automatic cycling will not work");

            return Commands.none();
        }
    }

    public static Command pathToLSource(KrakenSwerve swerve){
        return pathTo(swerve, "LSource");
    }

    public static Command pathToBarge(KrakenSwerve swerve){
        return pathTo(swerve, "Barge");
    }

    public static Command pathToRSource(KrakenSwerve swerve){
        return pathTo(swerve, "RSource");
    }

    public static Command pathToProcessor(KrakenSwerve swerve){
        return pathTo(swerve, "Processor");
    }

    public static Command pathTo1L(KrakenSwerve swerve) {
        return pathTo(swerve,"1L");
    }

    public static Command pathTo1R(KrakenSwerve swerve) {
        return pathTo(swerve,"1R");
    }

    public static Command pathTo2L(KrakenSwerve swerve) {
        return pathTo(swerve,"2L");
    }

    public static Command pathTo2R(KrakenSwerve swerve) {
        return pathTo(swerve,"2R");
    }

    public static Command pathTo3L(KrakenSwerve swerve) {
        return pathTo(swerve, "3L");
    }

    public static Command pathTo3R(KrakenSwerve swerve) {
        return pathTo(swerve,"3R");
    }

    public static Command pathTo4L(KrakenSwerve swerve) {
        return pathTo(swerve,"4L");
    }

    public static Command pathTo4R(KrakenSwerve swerve) {
        return pathTo(swerve,"4R");
    }

    public static Command pathTo5L(KrakenSwerve swerve) {
        return pathTo(swerve,"5L");
    }

    public static Command pathTo5R(KrakenSwerve swerve) {
        return pathTo(swerve,"5R");
    }

    public static Command pathTo6L(KrakenSwerve swerve) {
        return pathTo(swerve,"6L");
    }
    
    public static Command pathTo6R(KrakenSwerve swerve) {
        return pathTo(swerve,"6R");
    }
}
