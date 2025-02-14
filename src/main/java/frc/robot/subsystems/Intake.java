package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ElasticUtil;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
    MotionMagicVoltage armControlRequest = new MotionMagicVoltage(0.0);
    TalonFX intakeMotor;
    TalonFX wheelMotor;
    DigitalInput photoelectricSensor = new DigitalInput(kPhotoelectricSensorID);
    private final StatusSignal<Angle> intakeMotorPosSignal;

    public Intake() {
        intakeMotor = new TalonFX(kIntakeMotorID);
        intakeMotorPosSignal = intakeMotor.getPosition();
        intakeMotorPosSignal.setUpdateFrequency(100);
        intakeMotor.optimizeBusUtilization();
        ElasticUtil.checkStatus(intakeMotor.getConfigurator().apply(kIntakeMotorConfig));

        wheelMotor = new TalonFX(kWheelMotorID);
        ElasticUtil.checkStatus(wheelMotor.getConfigurator().apply(kWheelMotorConfig));
        wheelMotor.optimizeBusUtilization();
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
            Commands.waitSeconds(kEjectDelay),
            runOnce(()-> wheelMotor.set(0.0))
        );
    }

    public boolean hasCoral() {
        return photoelectricSensor.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Has Coral", photoelectricSensor.get());
    }
}