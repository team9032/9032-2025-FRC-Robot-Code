package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.automation.ButtonBoardHandler.AlgaeScorePath;
import frc.robot.automation.ButtonBoardHandler.ReefLevel;
import frc.robot.utils.ElasticUtil;

import static frc.robot.Constants.EndEffectorConstants.*;

import java.util.Map;
import java.util.function.Supplier;

public class EndEffector extends SubsystemBase {
    private final TalonFX rollerMotor;

    private final StatusSignal<Current> rollerCurrentSignal;

    private final TorqueCurrentFOC rollerCurrentRequest = new TorqueCurrentFOC(0);

    public EndEffector() {
        rollerMotor = new TalonFX(kEndEffectorRollerMotorID);

        rollerCurrentSignal = rollerMotor.getTorqueCurrent();
        rollerCurrentSignal.setUpdateFrequency(50);
        rollerMotor.optimizeBusUtilization();
        
        ElasticUtil.checkStatus(rollerMotor.getConfigurator().apply(kEndEffectorRollerMotorConfig));
    }

    private Command setRollerMotorPower(double power) {
        return runOnce(() -> rollerMotor.set(power));
    }

    private Command setRollerMotorCurrent(double current) {
        return runOnce(() -> rollerMotor.setControl(rollerCurrentRequest.withOutput(current)));
    }

    public Command placeCoralOnBranch(Supplier<ReefLevel> reefLevelSup) {
        return new SelectCommand<ReefLevel>(
            Map.ofEntries (
                Map.entry(ReefLevel.NONE, Commands.none()),
                Map.entry(ReefLevel.L1, Commands.none()),
                Map.entry(ReefLevel.L2, placeCoralOnMiddleBranch()),
                Map.entry(ReefLevel.L3, placeCoralOnMiddleBranch()),
                Map.entry(ReefLevel.L4, placeCoralOnL4())
            ),
            reefLevelSup
        );
    }

    private Command placeCoralOnMiddleBranch() {
        return Commands.sequence(
            setRollerMotorPower(kCoralOuttakePower),
            Commands.waitSeconds(kCoralOuttakeWait),
            setRollerMotorPower(0.0)
        );
    }

    public Command placeCoralInTrough() {
        return Commands.sequence(
            setRollerMotorPower(kCoralOuttakeToTroughPower),
            Commands.waitSeconds(kCoralOuttakeWaitToTrough),
            setRollerMotorPower(0.0)
        );
    }

    private Command placeCoralOnL4() {
        return Commands.sequence(
            setRollerMotorPower(kCoralOuttakeToL4Power),
            Commands.waitSeconds(kCoralOuttakeWaitToL4),
            setRollerMotorPower(0.0)
        );
    }

    public Command pickupCoralFromCradle() { 
        return Commands.sequence(
            setRollerMotorCurrent(kHoldCoralCurrent),
            Commands.waitUntil(this::hasCoral)
        );
    }

    public Command intakeAlgae() {
        return Commands.sequence(
            setRollerMotorCurrent(kHoldAlgaeCurrent),
            Commands.waitUntil(this::hasAlgae)
        );
    }

    public Command outtakeProcessorAlgae() {
        return Commands.sequence(
            setRollerMotorPower(kProcessorOuttakePower), 
            Commands.waitSeconds(kAlgaeOuttakeWait),
            setRollerMotorPower(0.0)
        );
    }

    public Command outtakeNetAlgae() {
        return Commands.sequence(
            setRollerMotorPower(kNetOuttakePower),
            Commands.waitSeconds(kAlgaeOuttakeWait),
            setRollerMotorPower(0.0)
        );
    }

    public Command stopRollers() {
        return setRollerMotorPower(0.0);
    }

    public boolean hasAlgae() {
        return rollerCurrentSignal.getValueAsDouble() < kHasAlgaeCurrent;
    }

    public boolean hasCoral() {
        return rollerCurrentSignal.getValueAsDouble() < kHasCoralCurrent && rollerCurrentSignal.getValueAsDouble() > kHasAlgaeCurrent;
    }

    @Override
    public void periodic() {
        rollerCurrentSignal.refresh();

        SmartDashboard.putBoolean("Has Algae", hasAlgae());
        SmartDashboard.putBoolean("End Effector Has Coral", hasCoral());
    }
}