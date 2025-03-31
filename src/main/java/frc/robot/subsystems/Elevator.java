package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.ElasticUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ElevatorConfigs.*;

public class Elevator extends SubsystemBase {
    private final TalonFX elevatorMotor;
    private final TalonFX elevatorMotorFollower;

    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);
    private final Follower followerMotorControl;
    private final StatusSignal<Angle> elevatorPosSignal;

    public static boolean elevatorOverSlowModeHeight;

    public Elevator() {
        //The leader of a follower cannot have its bus utilization optimized
        elevatorMotor = new TalonFX(kFrontElevatorID);
        elevatorPosSignal = elevatorMotor.getPosition();
        elevatorPosSignal.setUpdateFrequency(100);
        ElasticUtil.checkStatus(elevatorMotor.getConfigurator().apply(kElevatorConfig));

        elevatorMotorFollower = new TalonFX(kBackElevatorID);
        elevatorMotorFollower.optimizeBusUtilization();
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

    public boolean atTrough() {
        return atPosition(kElevatorTrough);
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
    
    public boolean atSetpoint() {
        return atPosition(motionMagic.Position);
    }

    public boolean overIndexPosition() {
        return elevatorPosSignal.getValueAsDouble() > (kElevatorOverIndexer - kElevatorTolerance);
    }

    public Command holdPosition() {
        return runOnce(() -> moveElevator(elevatorPosSignal.getValueAsDouble()));
    }

    public Command moveToStowPosition() {
        return runOnce(() -> moveElevator(kElevatorStow));
    }

    public Command moveToIndexerPosition() {
        return runOnce(() -> moveElevator(kElevatorIndexerPos));
    }

    public Command moveToSourcePosition() {
        return runOnce(() -> moveElevator(kElevatorSource));
    }

    public Command moveToLowAlgaePosition() {
        return runOnce(() -> moveElevator(kElevatorLowAlgae));
    }

    public Command moveToHighAlgaePosition() {
        return runOnce(() -> moveElevator(kElevatorHighAlgae));
    }

    public Command moveToTroughPosition() {
        return runOnce(() -> moveElevator(kElevatorTrough));
    }

    public Command moveToL1Position() {
        return runOnce(() -> moveElevator(kElevatorL1));
    }

    public Command moveToL2Position() {
        return runOnce(() -> moveElevator(kElevatorL2));
    }

    public Command moveToL3Position() {
        return runOnce(() -> moveElevator(kElevatorL3));
    }

    public Command moveToProcessorPosition() {
        return runOnce(() -> moveElevator(kElevatorProcessor));
    }

    public Command moveToNetPosition() {
        return runOnce(() -> moveElevator(kElevatorNet));
    }

    public Command moveToOverIndexerPosition() {
        return runOnce(() -> moveElevator(kElevatorOverIndexer));
    }

    public double getElevatorSwerveSpeedFactor() {
        if (elevatorPosSignal.getValueAsDouble() < kAutoSlowModeHeight) {
            return 0;
        } else {
            return ((-elevatorPosSignal.getValueAsDouble() + kElevatorSoftLimit.ForwardSoftLimitThreshold + kAutoSlowModeHeight) * kElevatorSwerveSpeedFactor) - 1.535; //arbitrary constant b/c thats how the math works, dont forget to change when tuning!!!!!!!!!!!!
        }
    }

    @Override
    public void periodic() {
        elevatorPosSignal.refresh();
        
        SmartDashboard.putBoolean("Elevator at setpoint", atSetpoint());
        SmartDashboard.putNumber("Swerve speed factor", getElevatorSwerveSpeedFactor());
    }
}