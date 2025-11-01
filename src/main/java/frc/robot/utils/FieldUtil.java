package frc.robot.utils;

import static frc.robot.Constants.LocalizationConstants.kBackReefTagsStartingID;
import static frc.robot.Constants.LocalizationConstants.kMaxReefTagID;
import static frc.robot.Constants.LocalizationConstants.kMinReefTagID;
import static frc.robot.Constants.LocalizationConstants.kReefCenter;
import static frc.robot.Constants.PathFollowingConstants.kAlgaeReefIntakeOffset;
import static frc.robot.Constants.PathFollowingConstants.kBargeAlignmentX;
import static frc.robot.Constants.PathFollowingConstants.kBargeMaxY;
import static frc.robot.Constants.PathFollowingConstants.kEndEffectorClearReefDistance;
import static frc.robot.Constants.PathFollowingConstants.kEndEffectorClearReefDistanceWithAlgae;
import static frc.robot.Constants.PathFollowingConstants.kLeftScoringOffset;
import static frc.robot.Constants.PathFollowingConstants.kPrepareForAlgaeIntakingReefDistance;
import static frc.robot.Constants.PathFollowingConstants.kPrepareForNetAlgaeScoringDistance;
import static frc.robot.Constants.PathFollowingConstants.kPrepareForScoringReefDistance;
import static frc.robot.Constants.PathFollowingConstants.kBargeAlignmentRotation;
import static frc.robot.Constants.PathFollowingConstants.kRightScoringOffset;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.localization.Localization;

public class FieldUtil {
    public static boolean shouldFlipCoordinates() {
        var optionalAlliance = DriverStation.getAlliance();

        return optionalAlliance.orElseGet(() -> Alliance.Blue).equals(Alliance.Red);
    }

    public static Pose2d flipPoseIfNeeded(Pose2d pose) {
        if (shouldFlipCoordinates())
            return FlippingUtil.flipFieldPose(pose);
        
        return pose; 
    }

    public static Translation2d flipTranslationIfNeeded(Translation2d translation) {
        if (shouldFlipCoordinates())
            return FlippingUtil.flipFieldPosition(translation);

        return translation;
    }

    public static double getRobotToReefDistance(Localization localization) {
        var reefCenter = flipTranslationIfNeeded(kReefCenter);

        return localization.getCurrentPose().getTranslation().getDistance(reefCenter);
    }

    public static boolean shouldPrepareToScoreCoral(Localization localization) {
        return getRobotToReefDistance(localization) < kPrepareForScoringReefDistance;
    }

    public static boolean shouldPrepareToIntakeAlgae(Localization localization) {
        return getRobotToReefDistance(localization) < kPrepareForAlgaeIntakingReefDistance;
    }

    public static boolean endEffectorCanClearReef(Localization localization) {
        return getRobotToReefDistance(localization) > kEndEffectorClearReefDistance;
    }

    public static boolean endEffectorWithAlgaeCanClearReef(Localization localization) {
        return getRobotToReefDistance(localization) > kEndEffectorClearReefDistanceWithAlgae;
    }

    public static boolean shouldPrepareToScoreNetAlgae(Localization localization) {
        var bargeLine = flipTranslationIfNeeded(new Translation2d(kBargeAlignmentX, 0));

        return Math.abs(localization.getCurrentPose().getTranslation().getX() - bargeLine.getX()) < kPrepareForNetAlgaeScoringDistance;
    }

    public static Pose2d getBargeAlignmentPose(Localization localization) {
        double currentY = localization.getCurrentPose().getY();

        var bargePoseFlipped = flipPoseIfNeeded(new Pose2d(kBargeAlignmentX, currentY, kBargeAlignmentRotation));
    
        double targetY = currentY;
        /* If we are not in front of the correct barge, align to the edge of the correct one */
        if (bargePoseFlipped.getY() < kBargeMaxY) 
            targetY = flipTranslationIfNeeded(new Translation2d(0, kBargeMaxY)).getY();

        return new Pose2d(bargePoseFlipped.getX(), targetY, bargePoseFlipped.getRotation());
    }

    public static Pose2d getClosestReefScoringLocation(Localization localization, boolean isLeftBranch) {
        var tagID = getClosestReefTagID(localization);
        
        return getReefScoringLocationFromTagID(localization, isLeftBranch, tagID);
    } 

    public static Pose2d getClosestReefAlgaeIntakeLocation(Localization localization) {
        var closestPose = flipPoseIfNeeded(localization.getTagPose(getClosestReefTagID(localization)));

        return closestPose.transformBy(kAlgaeReefIntakeOffset);
    } 

    public static boolean isClosestReefLocationHighAlgae(Localization localization) {
        var closestTagID = getClosestReefTagID(localization);

        return closestTagID % 2 == 0;//Even tags are high algae
    } 

    private static int getClosestReefTagID(Localization localization) {
        var currentPose = localization.getCurrentPose();

        /* Find the closest tag pose to the current pose */
        double closestPoseDistance = Double.MAX_VALUE;
        int closestPoseID = 0;
        for (int id = kMinReefTagID; id <= kMaxReefTagID; id++) {
            var tagPose = flipPoseIfNeeded(localization.getTagPose(id));

            double distance = currentPose.getTranslation().getDistance(tagPose.getTranslation());

            if (distance < closestPoseDistance) {
                closestPoseDistance = distance;
                closestPoseID = id;
            }
        }

        return closestPoseID;
    }

    public static Pose2d getReefScoringLocationFromTagID(Localization localization, boolean isLeftBranch, int tagID) {
        var tagPose = flipPoseIfNeeded(localization.getTagPose(tagID));

        /* Invert direction on the back reef faces so the perspective makes sense */
        if (tagID >= kBackReefTagsStartingID)
            return tagPose.transformBy(isLeftBranch ? kRightScoringOffset : kLeftScoringOffset);

        else 
            return tagPose.transformBy(isLeftBranch ? kLeftScoringOffset : kRightScoringOffset);
    }   

    public static Pose2d getReefScoringLocationOffsetFromTagID(Localization localization, boolean isLeftBranch, int tagID) {
        var tagPose = flipPoseIfNeeded(localization.getTagPose(tagID));

        /* Invert direction on the back reef faces so the perspective makes sense */
        if (tagID >= kBackReefTagsStartingID)
            return tagPose.transformBy(isLeftBranch ? kRightScoringOffset : kLeftScoringOffset).transformBy(new Transform2d(Units.inchesToMeters(6.0), 0, Rotation2d.kZero));

        else 
            return tagPose.transformBy(isLeftBranch ? kLeftScoringOffset : kRightScoringOffset).transformBy(new Transform2d(Units.inchesToMeters(6.0), 0, Rotation2d.kZero));
    }   
}
