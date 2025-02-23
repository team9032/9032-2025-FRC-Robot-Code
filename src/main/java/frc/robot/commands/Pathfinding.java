package frc.robot.commands;

import static frc.robot.Constants.PathplannerConfig.kDynamicPathConstraints;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.KrakenSwerve;
import frc.robot.utils.ElasticUtil;

public class Pathfinding {
    private Pathfinding() {}

    // private static Command pathTo(KrakenSwerve swerve, String pathName) {
    //     return Commands.defer(() -> getFollowingAndAlignmentCommand(swerve, pathName), Set.of(swerve));
    // }//TODO only needed if final alignment

    private static Command pathTo(KrakenSwerve swerve, String pathName) {
        try {
            PathPlannerPath pathToFollow = PathPlannerPath.fromPathFile(pathName);

            return AutoBuilder.pathfindThenFollowPath(pathToFollow, kDynamicPathConstraints);
                //.andThen(new AlignWithPose(swerve, endPose))//TODO needed?
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
