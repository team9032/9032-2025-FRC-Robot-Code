package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ElasticUtil;

import static frc.robot.Constants.IndexerConstants.*;

public class Indexer extends SubsystemBase {
    private final TalonFX rollerMotor;
    private final TimeOfFlight coralSensor;

    public Indexer() {
        rollerMotor = new TalonFX(kRollerMotorID);
        coralSensor = new TimeOfFlight(kSensorPort);
        ElasticUtil.checkStatus(rollerMotor.getConfigurator().apply(kRollerMotorConfig));
    }

    public Command spinRollers() {
        return runOnce(() -> rollerMotor.set(kRollerMotorPower));
    }

    public Command stopRollers() {
        return runOnce(() -> rollerMotor.set(0));
    }

    public boolean hasCoral() {
        return coralSensor.getRange() < kHasCoralRange;
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