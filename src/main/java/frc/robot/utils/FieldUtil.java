package frc.robot.utils;

import static frc.robot.Constants.DriverConstants.kCloseDistanceToReefCenter;
import static frc.robot.Constants.LocalizationConstants.kReefCenter;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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

    public static boolean isCloseToReef(Translation2d translation) {
        var reefCenter = flipTranslationIfNeeded(kReefCenter);

        return translation.getDistance(reefCenter) < kCloseDistanceToReefCenter;
    }
}
