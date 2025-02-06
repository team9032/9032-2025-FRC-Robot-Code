package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.ElasticUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ElevatorConfigs.*;

public class Elevator extends SubsystemBase {
    private final TalonFX elevatorMotor;
    private final TalonFX elevatorMotorFollower;

    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);
    private final Follower followerMotorControl;

    public Elevator() {
        elevatorMotor = new TalonFX(kMotor1ID);
        ElasticUtil.checkStatus(elevatorMotor.getConfigurator().apply(kELevatorMotorConfig));

        elevatorMotorFollower = new TalonFX(kMotor2ID);
        ElasticUtil.checkStatus(elevatorMotorFollower.getConfigurator().apply(kELevatorMotorConfig));

        followerMotorControl = new Follower(elevatorMotor.getDeviceID(), true);
        elevatorMotorFollower.setControl(followerMotorControl);
    }

    private void moveElevator(double pos) {
        elevatorMotor.setControl(motionMagic.withPosition(pos));
    }
    
    public boolean atSetPoint() {
        return MathUtil.isNear(motionMagic.Position, elevatorMotor.getPosition().getValueAsDouble(), kElevatorTolerance);
    }

    public Command moveToIndexerPosition() {
        return runOnce(() -> moveElevator(kElevatorIndexerPos));
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

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Elevator at setpoint", atSetPoint());
    }
}