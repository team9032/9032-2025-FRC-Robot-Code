package frc.robot.pathing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public record BezierTrajectoryState(Pose2d targetPose, Translation2d fieldCentricVelocity, Translation2d fieldCentricAcceleration) {}
