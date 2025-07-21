package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.automation.ButtonBoardHandler.ReefLevel;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.ElasticUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DriverConstants.kCANBusName;
import static frc.robot.Constants.ElevatorConfigs.*;

import java.util.Map;
import java.util.function.Supplier;

public class Elevator extends SubsystemBase {
    private final TalonFX elevatorMotor;
    private final TalonFX elevatorMotorFollower;

    private final MotionMagicExpoVoltage motionMagic = new MotionMagicExpoVoltage(0);
    private final Follower followerMotorControl;
    private final StatusSignal<Angle> elevatorPosSignal;

    public Elevator() {
        //The leader of a follower cannot have its bus utilization optimized
        elevatorMotor = new TalonFX(kFrontElevatorID, kCANBusName);
        elevatorPosSignal = elevatorMotor.getPosition();
        elevatorPosSignal.setUpdateFrequency(100);
        ElasticUtil.checkStatus(elevatorMotor.getConfigurator().apply(kElevatorConfig));

        elevatorMotorFollower = new TalonFX(kBackElevatorID, kCANBusName);
        ElasticUtil.checkStatus(elevatorMotorFollower.getConfigurator().apply(kElevatorConfig));

        followerMotorControl = new Follower(elevatorMotor.getDeviceID(), true);
        elevatorMotorFollower.setControl(followerMotorControl);
    }

    private void moveElevator(double pos) {
        elevatorMotor.setControl(motionMagic.withPosition(pos));
    }

    private boolean atPosition(double position) {
        return MathUtil.isNear(position, elevatorPosSignal.getValueAsDouble(), kElevatorTolerance);
    }

    public boolean atL1() {
        return atPosition(kElevatorL1);
    }

    public boolean atL2() {
        return atPosition(kElevatorL2);
    }

    public boolean atL3() {
        return atPosition(kElevatorL3);
    }

    public boolean atL4() {
        return atPosition(kElevatorL4);
    }
    
    public boolean atSetpoint() {
        return atPosition(motionMagic.Position);
    }

    public boolean overCradlePosition() {
        return elevatorPosSignal.getValueAsDouble() > (kElevatorOverCradle - kElevatorTolerance);
    }

    public boolean closeToNetPosition() {
        return elevatorPosSignal.getValueAsDouble() > kElevatorCloseToNet;
    }

    public Command holdPosition() {
        return runOnce(() -> moveElevator(elevatorPosSignal.getValueAsDouble()));
    }

    public Command moveToStowPosition() {
        return runOnce(() -> moveElevator(kElevatorStow));
    }

    public Command moveToCradlePosition() {
        return runOnce(() -> moveElevator(kElevatorCradlePos));
    }

    public Command moveToLowAlgaePosition() {
        return runOnce(() -> moveElevator(kElevatorLowAlgae));
    }

    public Command moveToHighAlgaePosition() {
        return runOnce(() -> moveElevator(kElevatorHighAlgae));
    }

    public Command moveToClimbPosition() {
        return runOnce(() -> moveElevator(kElevatorClimb));
    }

    public Command moveToCoralScoreLevel(Supplier<ReefLevel> reefLevelSup) {
        return new SelectCommand<ReefLevel>(
            Map.ofEntries(
                Map.entry(ReefLevel.NONE, Commands.none()),
                Map.entry(ReefLevel.L1, runOnce(() -> moveElevator(kElevatorL1))),
                Map.entry(ReefLevel.L2, runOnce(() -> moveElevator(kElevatorL2))),
                Map.entry(ReefLevel.L3, runOnce(() -> moveElevator(kElevatorL3))),
                Map.entry(ReefLevel.L4, runOnce(() -> moveElevator(kElevatorL4)))
            ),
            reefLevelSup
        );
    }

    public Command moveToProcessorPosition() {
        return runOnce(() -> moveElevator(kElevatorProcessor));
    }

    public Command moveToNetPosition() {
        return runOnce(() -> moveElevator(kElevatorNet));
    }

    public Command moveToOverCradlePosition() {
        return runOnce(() -> moveElevator(kElevatorOverCradle));
    }

    public Command moveToAlgaeGroundPosition() {
        return runOnce(() -> moveElevator(kElevatorAlgaeGround));
    }

    public boolean overHighAlgae() {
        return elevatorPosSignal.getValueAsDouble() > kElevatorOverHighAlgae;
    }

    public Command coast() {
        return runOnce(() -> elevatorMotor.setControl(new CoastOut()))
            .ignoringDisable(true);
    }

    @Override
    public void periodic() {
        elevatorPosSignal.refresh();

        SmartDashboard.putBoolean("Elevator At Setpoint", atSetpoint());
    }
}