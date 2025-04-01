package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ElasticUtil;

import static frc.robot.Constants.IndexerConstants.*;

public class Indexer extends SubsystemBase {
    private final TalonFX rollerMotor;

    public Indexer() {
        rollerMotor = new TalonFX(kIndexerRollerID);
        ElasticUtil.checkStatus(rollerMotor.getConfigurator().apply(kIndexerRollerConfig));
        
        rollerMotor.optimizeBusUtilization();
    }

    public Command spinRollers() {
        return runOnce(() -> rollerMotor.set(kIndexerRollerPower));
    }

    public Command stopRollers() {
        return runOnce(() -> rollerMotor.set(0));
    }

    public Command eject() {
        return runOnce(() -> rollerMotor.set(kIndexerEjectPower))
            .andThen(
                Commands.waitSeconds(kIndexerEjectWait),
                stopRollers()
            );
    }

    @Override
    public void periodic() {

    }
}