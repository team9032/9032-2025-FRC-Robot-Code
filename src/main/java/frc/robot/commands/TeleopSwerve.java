package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.KrakenSwerve;

import static frc.robot.Constants.DriverConstants.*;

public class TeleopSwerve extends Command {
    private final KrakenSwerve swerve;

    private final DoubleSupplier rotSup;
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;

    private final BooleanSupplier slowModeSup;

    public TeleopSwerve(KrakenSwerve swerve, DoubleSupplier rotSup, DoubleSupplier translationSup, DoubleSupplier strafeSup, BooleanSupplier slowModeSup) {
        this.swerve = swerve;

        this.rotSup = rotSup;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.slowModeSup = slowModeSup;

        addRequirements(swerve);
    }

    @Override
    public void execute() { 
        boolean slowMode = slowModeSup.getAsBoolean();
    
        double maxSpeed = slowMode ? kSlowSpeed : kMaxSpeed;
        double rotationRate = slowMode ? kSlowRotationRate : kRotationRate;

        /* Curve inputs to allow for more control closer to the lower range of the joystick */
        double translationVal = applyInputCurve(translationSup.getAsDouble());
        double strafeVal = applyInputCurve(strafeSup.getAsDouble());
        double rotationVal = applyInputCurve(rotSup.getAsDouble());

        /* Multiply by max speed to get the velocity values in m/s */
        translationVal *= maxSpeed;
        strafeVal *= maxSpeed;
        rotationVal *= rotationRate;

        swerve.drivetrain.setControl(
            kDriveRequest.withVelocityX(translationVal)
            .withVelocityY(strafeVal)
            .withRotationalRate(rotationVal) 
        );
    }

    private double applyInputCurve(double joystickInput) {
        return Math.copySign(Math.pow(joystickInput, 2), joystickInput);
    }
}
