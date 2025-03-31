package frc.robot.utils;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerve.KrakenSwerve;

public class LocalizationTrigger {
    private final KrakenSwerve swerve;
    private final Rectangle2d placeToTrigger;

    private final Trigger trigger;

    public LocalizationTrigger(KrakenSwerve swerve, Rectangle2d placeToTrigger) {
        this.swerve = swerve;
        this.placeToTrigger = placeToTrigger;

        this.trigger = new Trigger(this::isTriggered);
    }

    public Trigger getTrigger() {
        return trigger;
    }

    private boolean isTriggered() {
        Translation2d currentTranslation = swerve.drivetrain.getState().Pose.getTranslation();

        return placeToTrigger.contains(currentTranslation);
    }
}
