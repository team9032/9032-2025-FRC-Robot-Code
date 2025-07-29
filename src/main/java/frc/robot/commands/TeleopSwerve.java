package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.KrakenSwerve;

import static frc.robot.Constants.DriverConstants.*;
import static frc.robot.Constants.PathFollowingConstants.kAlignmentRotkD;
import static frc.robot.Constants.PathFollowingConstants.kAlignmentRotkP;

public class TeleopSwerve extends Command {
    private final KrakenSwerve swerve;

    private final DoubleSupplier rotSup;
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;

    private final PIDController headingController;

    public TeleopSwerve(KrakenSwerve swerve, DoubleSupplier rotSup, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
        this.swerve = swerve;

        this.rotSup = rotSup;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;

        headingController = new PIDController(kAlignmentRotkP, 0, kAlignmentRotkD);
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        headingController.reset();
        headingController.setSetpoint(swerve.getLocalization().getCurrentPose().getRotation().getRadians());
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

        double currentYaw = swerve.getLocalization().getCurrentPose().getRotation().getRadians();
        double currentRotRate = swerve.getLocalization().getCurrentVelocity().omegaRadiansPerSecond;

        /* Update setpoint if input is being applied or the chassis is still rotating to prevent jittering */
        if (Math.abs(rotationVal) > kJoystickDeadband * kRotationRate || Math.abs(currentRotRate) > kNoHeadingCorrectionRotRate) 
            headingController.setSetpoint(currentYaw);

        /* Maintain current heading if no input is being applied */
        else 
            rotationVal = headingController.calculate(currentYaw);

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
