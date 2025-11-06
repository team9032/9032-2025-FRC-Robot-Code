package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.KrakenSwerve;

import static frc.robot.Constants.DriverConstants.*;

public class TeleopSwerve extends Command {
    private final KrakenSwerve swerve;

    private final DoubleSupplier rotSup;
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;

    public TeleopSwerve(KrakenSwerve swerve, DoubleSupplier rotSup, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
        this.swerve = swerve;

        this.rotSup = rotSup;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;

        addRequirements(swerve);
    }

    @Override
    public void execute() {     
        double xCoord = translationSup.getAsDouble();
        double yCoord = strafeSup.getAsDouble();

        /* Convert cartesian joystick coordinates to polar */
        double angle = Math.atan2(yCoord, xCoord);
        double magnitude = Math.hypot(xCoord, yCoord);

        /* Curve magnitude to allow for more control closer to the lower range of the joystick */
        double curvedMagnitude = applyInputCurve(magnitude) * kMaxSpeed;
        double rotationMagnitude = applyInputCurve(rotSup.getAsDouble()) * kRotationRate;

        /* Convert polar to cartesian */
        double translationVal = curvedMagnitude * Math.cos(angle);
        double strafeVal = curvedMagnitude * Math.sin(angle);

        swerve.setControl(
            kDriveRequest.withVelocityX(translationVal)
            .withVelocityY(strafeVal)
            .withRotationalRate(rotationMagnitude) 
        );
    }

    private double applyInputCurve(double joystickInput) {
        return Math.copySign(Math.pow(joystickInput, 2), joystickInput);
    }
}
