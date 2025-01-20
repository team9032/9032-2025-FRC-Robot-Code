package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import static frc.robot.Constants.IntakeConstants.*;


import com.ctre.phoenix6.hardware.TalonFX;

public class Intake extends SubsystemBase{
    private final TalonFX extensionMotor;
    private final TalonFX wheelMotor;

    public Intake(){

        extensionMotor = new TalonFX(kExtensionMotorId);
        extensionMotor.getConfigurator().apply(kMotorConfig);

        wheelMotor = new TalonFX(36);
        wheelMotor.configFactoryDefault();

       
    }
   
    public Command resetCommand() {
        return runOnce(this::reset).withName("Reset");
    }

    private Command setExtensionPower(double power) {
        return runOnce(() -> extensionMotor.set(power));
    }

    public Command disableRollers() {
        return setRollerPower(0.0);
    }

    
    @Override
    public void periodic() {
        // lastUltrasonicDistance = ultrasonicFilter.calculate(ultrasonic.getRangeInches());
    }
}