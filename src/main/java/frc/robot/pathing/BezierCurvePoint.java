package frc.robot.pathing;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public record BezierCurvePoint(Translation2d location, Translation2d previousControlPoint, Translation2d nextControlPoint, Rotation2d targetRotation) {}