package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.ControlRequest;

public interface ElevatorIO {
    void setControl(ControlRequest controlRequest);

    double getPosition();

    default void periodic() {};
}
