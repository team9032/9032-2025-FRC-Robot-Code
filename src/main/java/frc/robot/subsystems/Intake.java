package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
    MotionMagicVoltage armControlRequest = new MotionMagicVoltage(0.0);
    TalonFX intakeMotor;
    TalonFX wheelMotor;
    DigitalInput photoelectricSensor = new DigitalInput(kPhotoelectricSensorID);


    public Intake() {
        intakeMotor = new TalonFX(kIntakeMotorID);
        intakeMotor.getConfigurator().apply(kIntakeMotorConfig);

        wheelMotor = new TalonFX(kWheelMotorID);
        wheelMotor.getConfigurator().apply(kWheelMotorConfig);
    }

    public Command returnToStowPosition() {
        return runOnce(() -> intakeMotor.setControl(armControlRequest.withPosition(kStowPosition)));
    }

    public Command intakeCoral() {
        return runOnce(() -> wheelMotor.set(kIntakePower));
    }

    public Command stopIntaking() {
        return runOnce(() -> wheelMotor.set(0.0));
    }

    public Command moveToGround() {
        return runOnce(() -> intakeMotor.setControl(armControlRequest.withPosition(kGroundPosition)));
    }

    public Command ejectCoral() {
        return Commands.sequence(
            runOnce(() -> wheelMotor.set(kEjectPower)),
            Commands.waitUntil(() -> !photoelectricSensor.get()),
            Commands.waitSeconds(kEjectDelay),
            runOnce(()-> wheelMotor.set(0.0))
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Has Coral", photoelectricSensor.get());
    }
}