package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ElasticUtil;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
    private final MotionMagicVoltage armControlRequest = new MotionMagicVoltage(0.0);

    private final TalonFX intakeArmMotor;
    private final TalonFX rollerMotor;

    private final TimeOfFlight intakeDistanceSensor = new TimeOfFlight(kIntakeDistSensorID);

    private final StatusSignal<Angle> intakeMotorPosSignal;

    public Intake() {
        intakeArmMotor = new TalonFX(kIntakeArmID);
        ElasticUtil.checkStatus(intakeArmMotor.getConfigurator().apply(kIntakeArmConfig));

        intakeMotorPosSignal = intakeArmMotor.getPosition();
        intakeMotorPosSignal.setUpdateFrequency(100);
        intakeArmMotor.optimizeBusUtilization();
        
        rollerMotor = new TalonFX(kIntakeRollerID);
        ElasticUtil.checkStatus(rollerMotor.getConfigurator().apply(kIntakeRollerConfig));
        
        rollerMotor.optimizeBusUtilization();
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

    public Command ejectCoral() {
        return Commands.sequence(
            runOnce(() -> rollerMotor.set(kEjectPower)),
            Commands.waitSeconds(kEjectDelay),
            runOnce(()-> rollerMotor.set(0.0))
        );
    }

    public boolean hasCoral() {
        return intakeDistanceSensor.getRange() < kCoralDetectionDistance; //TODO tune
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Has Coral", hasCoral());
    }
}