package frc.robot.utils;

import static frc.robot.Constants.DriverConstants.kCloseCoralDistanceToReefCenter;
import static frc.robot.Constants.LocalizationConstants.kBackReefTagsStartingID;
import static frc.robot.Constants.LocalizationConstants.kMaxReefTagID;
import static frc.robot.Constants.LocalizationConstants.kMinReefTagID;
import static frc.robot.Constants.LocalizationConstants.kReefCenter;
import static frc.robot.Constants.PathFollowingConstants.kAlgaeReefIntakeOffset;
import static frc.robot.Constants.PathFollowingConstants.kBargeAlignmentX;
import static frc.robot.Constants.PathFollowingConstants.kEndEffectorClearReefDistance;
import static frc.robot.Constants.PathFollowingConstants.kLeftScoringOffset;
import static frc.robot.Constants.PathFollowingConstants.kPrepareForAlgaeIntakingReefDistance;
import static frc.robot.Constants.PathFollowingConstants.kPrepareForNetAlgaeScoringDistance;
import static frc.robot.Constants.PathFollowingConstants.kPrepareForScoringReefDistance;
import static frc.robot.Constants.PathFollowingConstants.kRightScoringOffset;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.localization.Localization;

public class FieldUtil {
    public static Pose2d flipPoseIfNeeded(Pose2d pose) {
        var optionalAlliance = DriverStation.getAlliance();

        if (optionalAlliance.isPresent() && optionalAlliance.get().equals(Alliance.Red))
            return FlippingUtil.flipFieldPose(pose);
        
        return pose; 
    }

    public static Translation2d flipTranslationIfNeeded(Translation2d translation) {
        var optionalAlliance = DriverStation.getAlliance();

        if (optionalAlliance.isPresent() && optionalAlliance.get().equals(Alliance.Red))
            return FlippingUtil.flipFieldPosition(translation);

        return translation;
    }

    public static boolean isCoralCloseToReef(Translation2d translation) {
        var reefCenter = flipTranslationIfNeeded(kReefCenter);

        return translation.getDistance(reefCenter) < kCloseCoralDistanceToReefCenter;
    }

    public static boolean shouldPrepareToScoreCoral(Localization localization) {
        var reefCenter = flipTranslationIfNeeded(kReefCenter);

        return localization.getCurrentPose().getTranslation().getDistance(reefCenter) < kPrepareForScoringReefDistance;
    }

    public static boolean shouldPrepareToIntakeAlgae(Localization localization) {
        var reefCenter = flipTranslationIfNeeded(kReefCenter);

        return localization.getCurrentPose().getTranslation().getDistance(reefCenter) < kPrepareForAlgaeIntakingReefDistance;
    }

    public static boolean endEffectorCanClearReef(Localization localization) {
        var reefCenter = flipTranslationIfNeeded(kReefCenter);

        return localization.getCurrentPose().getTranslation().getDistance(reefCenter) > kEndEffectorClearReefDistance;
    }

    public static boolean shouldPrepareToScoreNetAlgae(Localization localization) {
        var bargeLine = flipTranslationIfNeeded(new Translation2d(kBargeAlignmentX, 0));

        return Math.abs(localization.getCurrentPose().getTranslation().getX() - bargeLine.getX()) < kPrepareForNetAlgaeScoringDistance;
    }

    public static Pose2d getClosestReefScoringLocation(Localization localization, boolean isLeftBranch) {
        var tag = getClosestReefTag(localization);
        var closestPoseID = tag.getSecond();
        var closestPose = tag.getFirst();

        /* Invert direction on the back reef faces so the perspective makes sense */
        if (closestPoseID >= kBackReefTagsStartingID)
            return closestPose.transformBy(isLeftBranch ? kRightScoringOffset : kLeftScoringOffset);

        else 
            return closestPose.transformBy(isLeftBranch ? kLeftScoringOffset : kRightScoringOffset);
    } 

    public static Pose2d getClosestReefAlgaeIntakeLocation(Localization localization) {
        var closestPose = getClosestReefTag(localization).getFirst();

        return closestPose.transformBy(kAlgaeReefIntakeOffset);
    } 

    public static boolean isClosestReefLocationHighAlgae(Localization localization) {
        var closestTagID = getClosestReefTag(localization).getSecond();

        return closestTagID % 2 == 0;//Even tags are high algae
    } 

    private static Pair<Pose2d, Integer> getClosestReefTag(Localization localization) {
        var currentPose = localization.getCurrentPose();

        /* Find the closest tag pose to the current pose */
        double closestPoseDistance = Double.MAX_VALUE;
        int closestPoseID = 0;
        Pose2d closestPose = Pose2d.kZero;
        for (int id = kMinReefTagID; id <= kMaxReefTagID; id++) {
            var tagPose = flipPoseIfNeeded(localization.getTagPose(id));

            double distance = currentPose.getTranslation().getDistance(tagPose.getTranslation());

            if (distance < closestPoseDistance) {
                closestPoseDistance = distance;
                closestPose = tagPose;
                closestPoseID = id;
            }
        }

        return Pair.of(closestPose, closestPoseID);
    }
}
