package frc.robot.pathing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public record CurvedPath(Pose2d finalPose, Rotation2d finalEntryAngle) {
    public static CurvedPath enterAtFinalRotation(Pose2d finalPose) {
        return new CurvedPath(finalPose, finalPose.getRotation().rotateBy(Rotation2d.k180deg));
    }

    public static CurvedPath createStraightPath(Pose2d initialPose, Pose2d finalPose) {
        var entryAngle = finalPose.getTranslation().minus(initialPose.getTranslation()).getAngle();

        return new CurvedPath(finalPose, entryAngle);
    }

    public Translation2d getPathDirection(Pose2d currentPose) {        
        return findDirectionFromTheta(getThetaInTargetSpace(currentPose)).rotateBy(finalEntryAngle);
    }  

    public double getRemainingPathDistance(Pose2d currentPose) {
        double straightDistance = currentPose.getTranslation().getDistance(finalPose.getTranslation());

        return findRemainingDistanceFromTheta(getThetaInTargetSpace(currentPose), straightDistance);
    }

    private double getThetaInTargetSpace(Pose2d currentPose) {
        var currentTranslationInTargetSpace = finalPose.getTranslation().minus(currentPose.getTranslation()).rotateBy(finalEntryAngle.unaryMinus());//TODO not sure
        
        return currentTranslationInTargetSpace.getAngle().getRadians();
    }

    private double findRemainingDistanceFromTheta(double theta, double straightDistance) {
        double b = Math.sqrt(1 + (theta * theta));
        double a = (straightDistance / 2.0) * b;

        if (theta == 0)
            return a;

        return a + ((straightDistance / (2.0 * theta)) * Math.log(Math.abs(theta + b)));
    }

    private Translation2d findDirectionFromTheta(double theta) {
        double x = (theta * -Math.sin(theta)) + Math.cos(theta);
        double y = (theta * Math.cos(theta)) + Math.sin(theta);

        var direction = new Translation2d(x, y);
        direction = direction.div(direction.getNorm());

        return direction;
    }
} 
