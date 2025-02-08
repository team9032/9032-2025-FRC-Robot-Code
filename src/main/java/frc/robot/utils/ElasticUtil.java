package frc.robot.utils;

import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Elastic;
import frc.lib.Elastic.Notification;
import frc.lib.Elastic.Notification.NotificationLevel;

/** Utility class for sending Elastic notifications and logging them to the console */
public class ElasticUtil {
    private static final Notification notification = new Notification();

    private ElasticUtil() {}

    public static void sendInfo(String title) {
        Elastic.sendNotification(notification.withTitle(title)); 

        System.out.println(title);
    }

    public static Command sendInfoCommand(String title) {
        return Commands.runOnce(() -> sendInfo(title));
    }

    public static void sendWarning(String title) {
        DriverStation.reportWarning(title, false);

        Elastic.sendNotification(
            notification.withTitle(title)
            .withLevel(NotificationLevel.WARNING)
        ); 
    }

    public static Command sendWarningCommand(String title) {
        return Commands.runOnce(() -> sendWarning(title));
    }

    public static void sendError(String title, String description) {
        DriverStation.reportError(title, true);

        Elastic.sendNotification(
            notification.withTitle(title)
            .withDescription(description)
            .withNoAutoDismiss()
            .withLevel(NotificationLevel.ERROR)
        );
    }

    public static Command sendErrorCommand(String title, String description) {
        return Commands.runOnce(() -> sendError(title, description));
    }

    /** Checks the status of a TalonFX */
    public static void checkStatus(StatusCode statusCode) {
        if(statusCode != StatusCode.OK) 
            sendError("TalonFX Error", "A motor will probably not work! Check the logs. Status code: " + statusCode.getName());
    }
}
