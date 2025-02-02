package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ArmConstants.*;

public class Arm extends SubsystemBase {
    private final TalonFX armMotor;
    private final MotionMagicVoltage armRequest = new MotionMagicVoltage(0);
    
    public Arm() {
        armMotor = new TalonFX(kArmMotorId);
        armMotor.getConfigurator().apply(kArmMotorConstants);
    }

    public boolean atSetpoint() {
        return MathUtil.isNear(armRequest.Position, armMotor.getPosition().getValueAsDouble(), kArmPositionTolerance);
    }

    public Command moveToIndexerPos() {
        return runOnce(() -> armMotor.setControl(armRequest.withPosition(kArmIndexerPos)));
    }

    public Command moveToTroughPos () {
        return runOnce(() -> armMotor.setControl(armRequest.withPosition(kArmTroughPos)));
    }

    public Command moveToLevel1Pos () {
        return runOnce(() -> armMotor.setControl(armRequest.withPosition(kArmLevel1Pos)));
    }

    public Command moveToLevel2Pos () {
        return runOnce(() -> armMotor.setControl(armRequest.withPosition(kArmLevel2Pos)));
    }

    public Command moveToLevel3Pos () {
        return runOnce(() -> armMotor.setControl(armRequest.withPosition(kArmLevel3Pos)));
    }

    public Command moveToHighAlgaePos () {
        return runOnce(() -> armMotor.setControl(armRequest.withPosition(kArmHighAlgaePos)));
    }
    
    public Command moveToLowAlgaePos () {
        return runOnce(() -> armMotor.setControl(armRequest.withPosition(kArmLowAlgaePos)));
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("At Setpoint", atSetpoint());
    }
}