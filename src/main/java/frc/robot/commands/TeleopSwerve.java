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
        /* Curve inputs to allow for more control closer to the lower range of the joystick */
        double translationVal = applyInputCurve(translationSup.getAsDouble());
        double strafeVal = applyInputCurve(strafeSup.getAsDouble());
        double rotationVal = applyInputCurve(rotSup.getAsDouble());

        /* Multiply by max speed to get the velocity values in m/s */
        translationVal *= kMaxSpeed;
        strafeVal *= kMaxSpeed;
        rotationVal *= kRotationRate;

        swerve.setControl(
            kDriveRequest.withVelocityX(translationVal)
            .withVelocityY(strafeVal)
            .withRotationalRate(rotationVal) 
        );
    }

    private double applyInputCurve(double joystickInput) {
        return Math.copySign(Math.pow(joystickInput, 2), joystickInput);
    }
}
