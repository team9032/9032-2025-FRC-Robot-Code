package frc.robot.utils;

import static frc.robot.Constants.DriverConstants.kCloseCoralDistanceToReefCenter;
import static frc.robot.Constants.LocalizationConstants.kBackReefTagsStartingID;
import static frc.robot.Constants.LocalizationConstants.kMaxReefTagID;
import static frc.robot.Constants.LocalizationConstants.kMinReefTagID;
import static frc.robot.Constants.LocalizationConstants.kReefCenter;
import static frc.robot.Constants.PathFollowingConstants.kEndEffectorClearReefDistance;
import static frc.robot.Constants.PathFollowingConstants.kLeftScoringOffset;
import static frc.robot.Constants.PathFollowingConstants.kPrepareForScoringReefDistance;
import static frc.robot.Constants.PathFollowingConstants.kRightScoringOffset;

import com.pathplanner.lib.util.FlippingUtil;

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

    public static boolean shouldPrepareToScore(Localization localization) {
        var reefCenter = flipTranslationIfNeeded(kReefCenter);

        return localization.getCurrentPose().getTranslation().getDistance(reefCenter) < kPrepareForScoringReefDistance;
    }

    public static boolean endEffectorCanClearReef(Localization localization) {
        var reefCenter = flipTranslationIfNeeded(kReefCenter);

        return localization.getCurrentPose().getTranslation().getDistance(reefCenter) > kEndEffectorClearReefDistance;
    }

    public static Pose2d getClosestReefScoringLocation(Localization localization, boolean isLeftBranch) {
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

        /* Invert direction on the back reef faces so the perspective makes sense */
        if (closestPoseID >= kBackReefTagsStartingID)
            return closestPose.transformBy(isLeftBranch ? kRightScoringOffset : kLeftScoringOffset);

        else 
            return closestPose.transformBy(isLeftBranch ? kLeftScoringOffset : kRightScoringOffset);
    } 
}
