package frc.robot.utils;

import static frc.robot.Constants.DriverConstants.kCloseDistanceToReefCenter;
import static frc.robot.Constants.LocalizationConstants.kReefCenter;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldUtil {
    public static boolean isCloseToReef(Translation2d translation) {
        var optionalAlliance = DriverStation.getAlliance();
        var reefCenter = kReefCenter;

        if (optionalAlliance.isPresent() && optionalAlliance.get().equals(Alliance.Red))
            reefCenter = FlippingUtil.flipFieldPosition(kReefCenter);

        return translation.getDistance(reefCenter) < kCloseDistanceToReefCenter;
    }
}
