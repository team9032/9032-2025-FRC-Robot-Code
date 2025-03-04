package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ElasticUtil;

import static frc.robot.Constants.IndexerConstants.*;

public class Indexer extends SubsystemBase {
    private final TalonFX rollerMotor;
    private final DigitalInput photoelectricSensor = new DigitalInput(kPhotoelectricSensorID);

    public Indexer() {
        rollerMotor = new TalonFX(kIndexerRollerID);
        ElasticUtil.checkStatus(rollerMotor.getConfigurator().apply(kRollerMotorConfig));
        rollerMotor.optimizeBusUtilization();
    }

    public Command spinRollers() {
        return runOnce(() -> rollerMotor.set(kRollerMotorPower));
    }

    public Command stopRollers() {
        return runOnce(() -> rollerMotor.set(0));
    }

    public boolean hasCoral() {
        return photoelectricSensor.get();
    }

    public Command spinRollersUntilCoralReceived() {
        return spinRollers()
            .until(() -> hasCoral())
            .andThen(stopRollers());
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Indexer has coral", hasCoral());
    }
}