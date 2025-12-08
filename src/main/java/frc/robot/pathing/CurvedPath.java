package frc.robot.pathing;

import static frc.robot.pathing.PathingConstants.kStraightDriveDistance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public record CurvedPath(Pose2d finalPose, Rotation2d finalEntryAngle, double endingSpeed) {
    public CurvedPath(Pose2d finalPose, Rotation2d finalEntryAngle) {
        this(finalPose, finalEntryAngle, 0.0);
    }

    public static CurvedPath enterAtFinalRotation(Pose2d finalPose) {
        return new CurvedPath(finalPose, finalPose.getRotation());
    }

    public static CurvedPath createStraightPath(Translation2d currentTranslation, Pose2d finalPose) {
        var entryAngle = finalPose.getTranslation().minus(currentTranslation).getAngle();

        return new CurvedPath(finalPose, entryAngle);
    }

    public Translation2d getPathDirection(Translation2d currentTranslation) {        
        double straightDistance = currentTranslation.getDistance(finalPose.getTranslation());

        if (straightDistance < kStraightDriveDistance) {
            var direction = finalPose.getTranslation().minus(currentTranslation);
            direction = direction.div(direction.getNorm());

            return direction;
        }

        return findDirectionFromTheta(getThetaInTargetSpace(currentTranslation)).rotateBy(finalEntryAngle);
    }  

    public double getRemainingPathDistance(Translation2d currentTranslation) {
        double straightDistance = currentTranslation.getDistance(finalPose.getTranslation());

        if (straightDistance < kStraightDriveDistance) 
            return straightDistance;

        return findRemainingDistanceFromTheta(getThetaInTargetSpace(currentTranslation), straightDistance);
    }

    private double getThetaInTargetSpace(Translation2d currentTranslation) {
        var currentTranslationInTargetSpace = finalPose.getTranslation().minus(currentTranslation).rotateBy(finalEntryAngle.unaryMinus());
        
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
