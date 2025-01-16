package frc.robot;
package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase{
    private final WPI_TalonFX extensionMotor;
    private final WPI_TalonFX wheelMotor;

    public Intake(){

        extensionMotor = new WPI_TalonFX(intakeArmConstants);
        extensionMotor.config();

        wheelMotor = new WPI_TalonFX(intakeRollerConstants);
        wheelMotor.config();

        intakeTab.add(extensionMotor).withPosition(0, 0).withSize(...);
        intakeTab.add(wheelMotor).withPosition(...).withSize(...);
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