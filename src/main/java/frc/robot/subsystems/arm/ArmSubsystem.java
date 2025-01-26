package frc.robot.subsystems.arm;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ArmConstants.*;

public class ArmSubsystem extends SubsystemBase {
    private final TalonFX armMotor = new TalonFX(kArmMotorId);
    private final MotionMagicVoltage armRequest = new MotionMagicVoltage(0);
    
    public ArmSubsystem() {
        armMotor.getConfigurator().apply(kArmMotorConstants);
    }

    public Command armToTroughPos () {
        return runOnce(() -> armMotor.setControl(armRequest.withPosition(kArmTroughPos)));
    }

    public Command armToLevel1Pos () {
        return runOnce(() -> armMotor.setControl(armRequest.withPosition(kArmLevel1Pos)));
    }

    public Command armToLevel2Pos () {
        return runOnce(() -> armMotor.setControl(armRequest.withPosition(kArmLevel2Pos)));
    }

    public Command armToLevel3Pos () {
        return runOnce(() -> armMotor.setControl(armRequest.withPosition(kArmLevel3Pos)));
    }
}