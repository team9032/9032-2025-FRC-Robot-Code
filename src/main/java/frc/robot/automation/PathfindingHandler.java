package frc.robot.automation;

import static frc.robot.Constants.PathplannerConfig.kDynamicPathConstraints;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public static Command pathToLSource() {
        return pathTo("LSource");
    }

    public static Command pathToBarge(){
        return pathTo("Barge");
    }

    public static Command pathToRSource(){
        return pathTo("RSource");
    }

    public static Command pathToProcessor(){
        return pathTo("Processor");
    }

    public static Command pathTo1L() {
        return pathTo("1L");
    }

    public static Command pathTo1R() {
        return pathTo("1R");
    }

    public static Command pathTo2L() {
        return pathTo("2L");
    }

    public static Command pathTo2R() {
        return pathTo("2R");
    }

    public static Command pathTo3L() {
        return pathTo("3L");
    }

    public static Command pathTo3R() {
        return pathTo("3R");
    }

    public static Command pathTo4L() {
        return pathTo("4L");
    }

    public static Command pathTo4R() {
        return pathTo("4R");
    }

    public static Command pathTo5L() {
        return pathTo("5L");
    }

    public static Command pathTo5R() {
        return pathTo("5R");
    }

    public static Command pathTo6L() {
        return pathTo("6L");
    }
    
    public static Command pathTo6R() {
        return pathTo("6R");
    }

    public static Command pathTo1A() {
        return pathTo("1A");
    }

    public static Command pathTo2A() {
        return pathTo("2A");
    }

    public static Command pathTo3A() {
        return pathTo("3A");
    }

    public static Command pathTo4A() {
        return pathTo("4A");
    }

    public static Command pathTo5A() {
        return pathTo("5A");
    }

    public static Command pathTo6A() {
        return pathTo("6A");
    } 
}
