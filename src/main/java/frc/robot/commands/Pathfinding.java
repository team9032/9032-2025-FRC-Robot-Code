package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.KrakenSwerve;

public class Pathfinding {
    private Pathfinding() {}

    public static Command pathTo1L(KrakenSwerve swerve) {
        PathPlannerPath pathToFollow = PathPlannerPath.fromPathFile("1L");

        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(pathToFollow, null);
    }
}
