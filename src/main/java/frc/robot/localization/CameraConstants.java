package frc.robot.localization;

import edu.wpi.first.math.geometry.Transform3d;

public record CameraConstants(String name, Transform3d robotToCameraTransform) {}