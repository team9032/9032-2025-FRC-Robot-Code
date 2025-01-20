package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeConstants.*;


import com.ctre.phoenix6.hardware.TalonFX;

public class Intake extends SubsystemBase{
    private final TalonFX extensionMotor;
    private final TalonFX wheelMotor;

    public Intake(){

        extensionMotor = new TalonFX(CANID);
        extensionMotor.config();

        wheelMotor = new TalonFX(CANID);
        wheelMotor.config();

       
    }
   
    public Command resetCommand() {
        return runOnce(this::reset).withName("Reset");
    }

    private Command setExtensionPower(double power) {
        return runOnce(() -> extensionMotor.set(power));
    }
    
    @Override
    public void periodic() {
        // lastUltrasonicDistance = ultrasonicFilter.calculate(ultrasonic.getRangeInches());
    }
}