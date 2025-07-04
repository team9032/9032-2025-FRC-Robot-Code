package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ElasticUtil;
import static frc.robot.Constants.ClimberConstants.*;

public class Climber extends SubsystemBase {
    private final TalonFX armMotor;
    private final PositionVoltage armRequest = new PositionVoltage(0);
    private final StatusSignal<Angle> armPosSignal;

    private final TalonFX intakeMotor;
    private final StatusSignal<Current> intakeCurrentSignal;

    public Climber() {
        armMotor = new TalonFX(kClimberArmID);
        
        armPosSignal = armMotor.getPosition();
        armPosSignal.setUpdateFrequency(50);
        armMotor.optimizeBusUtilization();

        ElasticUtil.checkStatus(armMotor.getConfigurator().apply(kClimberArmMotorConfig));

        intakeMotor = new TalonFX(kClimberIntakeID);
        
        intakeCurrentSignal = intakeMotor.getTorqueCurrent();
        intakeCurrentSignal.setUpdateFrequency(50);
        intakeMotor.optimizeBusUtilization();

        ElasticUtil.checkStatus(intakeMotor.getConfigurator().apply(kClimberIntakeMotorConfig));
    }

    public boolean hasCage() {
        return intakeCurrentSignal.getValueAsDouble() > kHasCageCurrent;
    }

    public boolean atSetpoint() {
        return MathUtil.isNear(armRequest.Position, armPosSignal.getValueAsDouble(), kClimberArmTolerance);
    }

    private Command moveClimber(double pos) {
        return runOnce(() -> armMotor.setControl(armRequest.withPosition(pos)))
            .andThen(Commands.waitUntil(this::atSetpoint));
    }

    private Command runIntake() {
        return startEnd(() -> intakeMotor.setVoltage(kClimberIntakeVolts), () -> intakeMotor.setVoltage(0));
    }

    public Command moveToStowPosition() {
        return moveClimber(kClimberStowPos);
    }

    private Command moveToClimbPosition() {
        return moveClimber(kClimberClimbPos);
    }

    private Command moveToIntakePosition() {
        return moveClimber(kClimberCageIntakePos);
    }

    public Command coastArm() {
        return runOnce(() -> armMotor.setControl(new CoastOut()))
            .ignoringDisable(true);
    }

    public Command intakeCageAndClimb() {
        return Commands.sequence(
            moveToIntakePosition(),
            runIntake()
                .withDeadline(
                    Commands.waitUntil(this::hasCage)
                        .andThen(Commands.waitSeconds(kCageIntakeDelay))
                ),
            moveToClimbPosition()
        );
    }

    @Override
    public void periodic() {
        intakeCurrentSignal.refresh();
        armPosSignal.refresh();

        SmartDashboard.putNumber("Climber Position", armPosSignal.getValueAsDouble());
        SmartDashboard.putBoolean("Has Cage", hasCage());
    }
}