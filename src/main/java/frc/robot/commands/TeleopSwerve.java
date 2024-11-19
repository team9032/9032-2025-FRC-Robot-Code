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
        swerve.driveTrain.setControl(
            kDriveRequest.withVelocityX(translationSup.getAsDouble() * kSpeedLimit)
            .withVelocityY(strafeSup.getAsDouble() * kSpeedLimit)
            .withRotationalRate(rotSup.getAsDouble() * 4 * Math.PI) 
        );
    }
}
