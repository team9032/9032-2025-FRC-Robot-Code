package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.automation.ButtonBoardHandler.ReefLevel;
import frc.robot.utils.ElasticUtil;

import static frc.robot.Constants.ArmConstants.*;

import java.util.Map;
import java.util.function.Supplier;

public class Arm extends SubsystemBase {
    private final TalonFX armMotor;
    private final CANcoder armEncoder;
    private final MotionMagicExpoVoltage armRequest = new MotionMagicExpoVoltage(0);
    private final StatusSignal<Angle> armPosSignal;
    
    public Arm() { 
        armEncoder = new CANcoder(kArmEncoderId);
        //TODO armEncoder.optimizeBusUtilization();
        ElasticUtil.checkStatus(armEncoder.getConfigurator().apply(kArmEncoderConfig));

        armMotor = new TalonFX(kArmMotorId);
        
        armPosSignal = armMotor.getPosition();
        armPosSignal.setUpdateFrequency(100);
        //armMotor.optimizeBusUtilization();

        ElasticUtil.checkStatus(armMotor.getConfigurator().apply(kArmMotorConstants));
    }

    public Command holdPosition() {
        return runOnce(() -> armMotor.setControl(armRequest.withPosition(getPosition())));
    }

    public boolean atL1() {
        return atPosition(kArmL1Pos);
    }
    
    public boolean atL2() {
        return atPosition(kArmL2Pos);
    }

    public boolean atL3() {
        return atPosition(kArmL3Pos);
    }

    public boolean atL4() {
        return atPosition(kArmL4Pos);
    }

    public boolean atCoralPreparedToScorePos() {
        return atPosition(kArmCoralPreparedToScorePos);
    }

    private double getPosition() {
        return armPosSignal.getValueAsDouble();
    }

    private boolean atPosition(double position) {
        return MathUtil.isNear(position, getPosition(), kArmPositionTolerance);
    }

    public boolean atSetpoint() {
        return atPosition(armRequest.Position);
    }

    public boolean overIntake() {
        return getPosition() <= kArmOverIntakePos;
    }

    public boolean closeToCradlePosition() {
        return getPosition() > kArmCradlePos - (kArmPositionTolerance * 10.0);
    }

    public Command moveToPreparedToScoreCoralPos() {
        return runOnce(() -> armMotor.setControl(armRequest.withPosition(kArmCoralPreparedToScorePos)));
    }

    public Command moveToStowPos() {
        return runOnce(() -> armMotor.setControl(armRequest.withPosition(kArmStowPos)));
    }

    public Command moveToCradlePos() {
        return runOnce(() -> armMotor.setControl(armRequest.withPosition(kArmCradlePos)));
    }

    public Command moveToClimbPos() {
        return runOnce(() -> armMotor.setControl(armRequest.withPosition(kArmClimbPos)));
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

    public Command moveToAlgaeGroundPos() {
        return runOnce(() -> armMotor.setControl(armRequest.withPosition(kArmAlgaeGroundPos)));
    }

    public Command moveToCoralScoreLevel(Supplier<ReefLevel> reefLevelSup) {
        return new SelectCommand<ReefLevel>(
            Map.ofEntries (
                Map.entry(ReefLevel.NONE, Commands.none()),
                Map.entry(ReefLevel.L1, runOnce(() -> armMotor.setControl(armRequest.withPosition(kArmL1Pos)))),
                Map.entry(ReefLevel.L2, runOnce(() -> armMotor.setControl(armRequest.withPosition(kArmL2Pos)))),
                Map.entry(ReefLevel.L3, runOnce(() -> armMotor.setControl(armRequest.withPosition(kArmL3Pos)))),
                Map.entry(ReefLevel.L4, runOnce(() -> armMotor.setControl(armRequest.withPosition(kArmL4Pos))))
            ),
            reefLevelSup
        );
    }

    public Command coast() {
        return runOnce(() -> armMotor.setControl(new CoastOut()))
            .ignoringDisable(true);
    }

    @Override
    public void periodic() {
        armPosSignal.refresh();

        SmartDashboard.putBoolean("At Setpoint", atSetpoint());
        SmartDashboard.putNumber("End effector arm position", getPosition());
    }
}