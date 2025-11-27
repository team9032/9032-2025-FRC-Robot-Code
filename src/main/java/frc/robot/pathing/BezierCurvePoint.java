package frc.robot.pathing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
//TODO target rotation is only used for the endpoint
public record BezierCurvePoint(Pose2d targetPose, Translation2d previousControlPoint, Translation2d nextControlPoint) {
    public static BezierCurvePoint ofControlLengths(Pose2d targetPose, double previousControlLength, double nextControlLength, Rotation2d controlRotation) {
        var previousControlPoint = new Pose2d(targetPose.getTranslation(), controlRotation)
            .transformBy(new Transform2d(-previousControlLength, 0, Rotation2d.kZero));

        var nextControlPoint = new Pose2d(targetPose.getTranslation(), controlRotation)
            .transformBy(new Transform2d(nextControlLength, 0, Rotation2d.kZero));

        return new BezierCurvePoint(targetPose, previousControlPoint.getTranslation(), nextControlPoint.getTranslation());
    }
}