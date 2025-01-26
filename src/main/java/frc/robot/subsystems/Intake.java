package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
    private final MotionMagicVoltage armControlRequest = new MotionMagicVoltage(0.0);

    private final TalonFX armMotor;
    private final TalonFX wheelMotor;

    public Intake() {
        armMotor = new TalonFX(kExtensionMotorID);
        armMotor.getConfigurator().apply(kArmMotorConfig);

        wheelMotor = new TalonFX(kWheelMotorID);
        wheelMotor.getConfigurator().apply(kWheelMotorConfig);

    }

    public Command returnToStowPosition() {
        return runOnce(() -> armMotor.setControl(armControlRequest.withPosition(kStowPosition)));
    }

    public Command intakeCoral() {
        return runOnce(() -> wheelMotor.set(kIntakePower));
    }

    public Command moveToGround() {
        return runOnce(() -> armMotor.setControl(armControlRequest.withPosition(kGroundPosition)));
    }

    public Command ejectCoral() {
        return runOnce(() -> wheelMotor.set(kEjectPower));
    }

    @Override
    public void periodic() {

    }
}