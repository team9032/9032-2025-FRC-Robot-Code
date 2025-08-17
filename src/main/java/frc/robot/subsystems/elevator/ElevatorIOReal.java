package frc.robot.subsystems.elevator;

import static frc.robot.Constants.DriverConstants.kCANBusName;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import frc.robot.utils.ElasticUtil;

public class ElevatorIOReal implements ElevatorIO {
    private final TalonFX elevatorMotor;
    private final TalonFX elevatorMotorFollower;

    private final StatusSignal<Angle> elevatorPosSignal;

    public ElevatorIOReal() {
        elevatorMotor = new TalonFX(kFrontElevatorID, kCANBusName);
        elevatorPosSignal = elevatorMotor.getPosition();
        elevatorPosSignal.setUpdateFrequency(100);
        ElasticUtil.checkStatus(elevatorMotor.getConfigurator().apply(kElevatorConfig));

        elevatorMotorFollower = new TalonFX(kBackElevatorID, kCANBusName);
        ElasticUtil.checkStatus(elevatorMotorFollower.getConfigurator().apply(kElevatorConfig));

        var followerMotorControl = new Follower(elevatorMotor.getDeviceID(), true);
        elevatorMotorFollower.setControl(followerMotorControl);
    }

    @Override
    public void setControl(ControlRequest controlRequest) {
        elevatorMotor.setControl(controlRequest);
    }

    @Override
    public double getPosition() {
        return elevatorPosSignal.getValueAsDouble();
    }

    @Override
    public void periodic() {
        elevatorPosSignal.refresh();
    }
}
