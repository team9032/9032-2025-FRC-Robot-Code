package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.swerve.KrakenSwerve;

public class DriveToPose extends DriveToMovingPose {
    public DriveToPose(KrakenSwerve swerve, Pose2d targetPose) {
        super(swerve, () -> targetPose);
    }
}