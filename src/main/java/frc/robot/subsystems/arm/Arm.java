package frc.robot.subsystems.arm;

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

import static frc.robot.subsystems.arm.ArmConstants.*;
import static frc.robot.Constants.DriverConstants.kCANBusName;

import java.util.Map;
import java.util.function.Supplier;

public class Arm extends SubsystemBase {
    private final TalonFX armMotor;
    private final CANcoder armEncoder;
    private final MotionMagicExpoVoltage armRequest = new MotionMagicExpoVoltage(0);
    private final StatusSignal<Angle> armPosSignal;
    
    public Arm() { 
        armEncoder = new CANcoder(kArmEncoderId, kCANBusName);
        ElasticUtil.checkStatus(armEncoder.getConfigurator().apply(kArmEncoderConfig));

        armMotor = new TalonFX(kArmMotorId, kCANBusName);
        
        armPosSignal = armMotor.getPosition();
        armPosSignal.setUpdateFrequency(100);

        ElasticUtil.checkStatus(armMotor.getConfigurator().apply(kArmMotorConstants));
    }

    public Command holdPosition() {
        return runOnce(() -> armMotor.setControl(armRequest.withPosition(getPosition())));
    }
    
    public boolean atL2ScorePos() {
        return atPosition(kArmL2ScorePos);
    }

    public boolean atL3ScorePos() {
        return atPosition(kArmL3ScorePos);
    }

    public boolean atL4ScorePos() {
        return atPosition(kArmL4ScorePos);
    }

    public boolean overCradle() {
        return armPosSignal.getValueAsDouble() > kArmOverCradlePos;
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

    public Command moveToPreparedToScoreCoralPos() {
        return runOnce(() -> armMotor.setControl(armRequest.withPosition(kArmCoralPreparedToScorePos)));
    }

    public Command moveToPreparedToScoreLowL1Pos() {
        return runOnce(() -> armMotor.setControl(armRequest.withPosition(kArmLowL1PreparedToScorePos)));
    }

    public Command moveToPreparedToScoreHighL1Pos() {
        return runOnce(() -> armMotor.setControl(armRequest.withPosition(kArmHighL1PreparedToScorePos)));
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

    public Command moveToReefBranchScorePos(Supplier<ReefLevel> reefLevelSup) {
        return new SelectCommand<ReefLevel>(
            Map.ofEntries (
                Map.entry(ReefLevel.NONE, Commands.none()),
                Map.entry(ReefLevel.L1, Commands.none()),
                Map.entry(ReefLevel.L2, runOnce(() -> armMotor.setControl(armRequest.withPosition(kArmL2ScorePos)))),
                Map.entry(ReefLevel.L3, runOnce(() -> armMotor.setControl(armRequest.withPosition(kArmL3ScorePos)))),
                Map.entry(ReefLevel.L4, runOnce(() -> armMotor.setControl(armRequest.withPosition(kArmL4ScorePos))))
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
        SmartDashboard.putNumber("End Effector Arm Position", getPosition());
    }
}