package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ElasticUtil;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
    private final MotionMagicVoltage armControlRequest = new MotionMagicVoltage(0.0);

    private final TalonFX intakeArmMotor;
    private final TalonFX rollerMotor;

    private final StatusSignal<Angle> armMotorPosSignal;

    public Intake() {
        intakeArmMotor = new TalonFX(kIntakeArmID);
        ElasticUtil.checkStatus(intakeArmMotor.getConfigurator().apply(kIntakeArmConfig));

        armMotorPosSignal = intakeArmMotor.getPosition();
        armMotorPosSignal.setUpdateFrequency(50);
        intakeArmMotor.optimizeBusUtilization();
        
        rollerMotor = new TalonFX(kIntakeRollerID);
        ElasticUtil.checkStatus(rollerMotor.getConfigurator().apply(kIntakeRollerConfig));
        
        rollerMotor.optimizeBusUtilization();        
    }

    public Command holdPosition() {
        return runOnce(() -> intakeArmMotor.setControl(armControlRequest.withPosition(getArmPosition())));
    }

    private double getArmPosition() {
        return armMotorPosSignal.getValueAsDouble();
    }

    public Command returnToStowPosition() {
        return runOnce(() -> intakeArmMotor.setControl(armControlRequest.withPosition(kStowPosition)));
    }

    public Command intakeCoral() {
        return runOnce(() -> rollerMotor.set(kIntakePower));
    }

    public Command stopIntaking() {
        return runOnce(() -> rollerMotor.set(0.0));
    }

    public Command moveToGround() {
        return runOnce(() -> intakeArmMotor.setControl(armControlRequest.withPosition(kGroundPosition)));
    }

    public Command moveToEndEffectorMovePosition() {
        return runOnce(() -> intakeArmMotor.setControl(armControlRequest.withPosition(kEndEffectorMovePosition)));
    }

    public Command outtakeCoral() {
        return runOnce(() -> rollerMotor.set(kEjectPower));
    }

    public Command ejectCoral() {
        return Commands.sequence(
            runOnce(() -> rollerMotor.set(kEjectPower)),
            Commands.waitSeconds(kEjectDelay),
            runOnce(()-> rollerMotor.set(0.0))
        );
    }

    public boolean canRunRollers() {
        return getArmPosition() < kRunRollersPosition;
    }

    public boolean endEffectorCanMovePast() {
        return getArmPosition() < kEndEffectorMovePosition;
    }

    public boolean readyForClimbing() {
        return getArmPosition() > kRunRollersPosition;
    }

    public Command coast() {
        return runOnce(() -> intakeArmMotor.setControl(new CoastOut()))
            .ignoringDisable(true);
    }

    @Override
    public void periodic() {
        armMotorPosSignal.refresh();

        SmartDashboard.putBoolean("Can Run Rollers", canRunRollers());
    }
}