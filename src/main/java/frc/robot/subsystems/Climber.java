package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ElasticUtil;
import static frc.robot.Constants.ClimberConstants.*;

public class Climber extends SubsystemBase {
    private final TalonFX climberMotor;
    private final PositionVoltage motionMagic = new PositionVoltage(0);
    private final StatusSignal<Angle> climberPosSignal;

    public Climber() {
        climberMotor = new TalonFX(kMotorID);
        
        climberPosSignal = climberMotor.getPosition();
        climberPosSignal.setUpdateFrequency(100);
        climberMotor.optimizeBusUtilization();

        ElasticUtil.checkStatus(climberMotor.getConfigurator().apply(kClimberMotorConfig));
    }

    private void moveClimber(double pos) {
        climberMotor.setControl(motionMagic.withPosition(pos));
    }

    public Command moveClimberDown() {
        return runOnce(() -> moveClimber(kClimberDown));
    }

    public Command moveClimberUp() {
        return runOnce(() -> moveClimber(kClimberUp));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber position", climberPosSignal.getValueAsDouble());
    }
}