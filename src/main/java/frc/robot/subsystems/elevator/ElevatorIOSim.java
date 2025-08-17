package frc.robot.subsystems.elevator;

import static frc.robot.Constants.DriverConstants.kCANBusName;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import org.ironmaple.simulation.motorsims.SimulatedBattery;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.utils.ElasticUtil;

public class ElevatorIOSim implements ElevatorIO {
    private final ElevatorSim elevatorSimModel;

    private final TalonFX elevatorMotor;
    private final StatusSignal<Angle> elevatorPosSignal;

    public ElevatorIOSim() {
        elevatorSimModel = new ElevatorSim(
            DCMotor.getKrakenX60Foc(2),
            kElevatorConfig.Feedback.SensorToMechanismRatio,
            Units.lbsToKilograms(10.0),
            /*Units.inchesToMeters(1.0)*/1.0,
            0.0,
            2.0,
            true,
            0.0
        );

        elevatorMotor = new TalonFX(kFrontElevatorID, kCANBusName);
        elevatorPosSignal = elevatorMotor.getPosition();
        elevatorPosSignal.setUpdateFrequency(100);
        ElasticUtil.checkStatus(elevatorMotor.getConfigurator().apply(kElevatorConfig));

        SimulatedBattery.addElectricalAppliances(() -> elevatorMotor.getSupplyCurrent().getValue());
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
        elevatorSimModel.update(0.02);

        var motorSimState = elevatorMotor.getSimState();
        motorSimState.Orientation = ChassisReference.Clockwise_Positive;
        motorSimState.setRawRotorPosition(elevatorSimModel.getPositionMeters() * kElevatorConfig.Feedback.SensorToMechanismRatio);//TODO these units are broken
        motorSimState.setRotorVelocity(elevatorSimModel.getVelocityMetersPerSecond() * kElevatorConfig.Feedback.SensorToMechanismRatio);
        motorSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

        elevatorSimModel.setInputVoltage(motorSimState.getMotorVoltage());

        elevatorPosSignal.refresh();
    }
}
