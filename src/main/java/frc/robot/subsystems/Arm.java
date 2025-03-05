package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ElasticUtil;

import static frc.robot.Constants.ArmConstants.*;

public class Arm extends SubsystemBase {
    private final TalonFX armMotor;
    private final MotionMagicVoltage armRequest = new MotionMagicVoltage(0);
    private final StatusSignal<Angle> armPosSignal;
    private final DutyCycleEncoder armEncoder;
    
    public Arm() { 
        armMotor = new TalonFX(kArmMotorId);
        
        armPosSignal = armMotor.getPosition();
        armPosSignal.setUpdateFrequency(100);
        armMotor.optimizeBusUtilization();

        armEncoder = new DutyCycleEncoder(kArmEncoderPort, kArmEncoderRange, kArmEncoderZeroPos);

        ElasticUtil.checkStatus(armMotor.getConfigurator().apply(kArmMotorConstants));

        armMotor.setPosition(getArmAbsolutePosition());
    }

    public boolean atSetpoint() {
        return MathUtil.isNear(armRequest.Position, armPosSignal.getValueAsDouble(), kArmPositionTolerance);
    }

    public double getArmAbsolutePosition() {
        return armEncoder.get();
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

    public Command moveToNetPos() {
        return runOnce(() -> armMotor.setControl(armRequest.withPosition(kArmNetPos)));
    }

    public Command moveToProcessorPos() {
        return runOnce(() -> armMotor.setControl(armRequest.withPosition(kArmProcessorPos)));
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("At Setpoint", atSetpoint());
        SmartDashboard.putNumber("End effector arm absolute position", getArmAbsolutePosition());
        SmartDashboard.putNumber("End effector arm relative position", armPosSignal.getValueAsDouble());
    }
}