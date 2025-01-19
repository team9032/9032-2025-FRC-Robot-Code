package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimberConstants.*;

import java.util.function.DoubleSupplier;


public class Climber extends SubsystemBase {

    private TalonFX climberMotor;
    public Climber(){
        climberMotor = new TalonFX(motorID);
    }

    public Command setPower(DoubleSupplier pressure){
        return runOnce(() -> climberMotor.set(pressure.getAsDouble()));
    }

}