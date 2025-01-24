package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase{
    private final MotionMagicVoltage armControlRequest = new MotionMagicVoltage(0.0);

    private final TalonFX armMotor;
    private final TalonFX wheelMotor;

    public Intake(){
        //Call arm motor, use motion magic

        armMotor = new TalonFX(kArmMotorId);
        armMotor.getConfigurator().apply(kMotorConfig);

        var talonFXConfigs = new TalonFXConfiguration();        

        wheelMotor = new TalonFX(kWheelMotorId);
        wheelMotor.getConfigurator().apply(kMotorConfig);
       
    }

    public Command moveToGround() {
        return runOnce(() -> armMotor.setControl(armControlRequest.withPosition(28828)));
    }
    
    @Override
    public void periodic() {
        // lastUltrasonicDistance = ultrasonicFilter.calculate(ultrasonic.getRangeInches());
    }
}