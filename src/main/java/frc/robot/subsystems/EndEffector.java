package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
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

    public EndEffector() {
        rollerMotor = new TalonFX(kEndEffectorRollerMotorID);

        rollerCurrentSignal = rollerMotor.getStatorCurrent();
        rollerCurrentSignal.setUpdateFrequency(100);
        rollerMotor.optimizeBusUtilization();
        
        ElasticUtil.checkStatus(rollerMotor.getConfigurator().apply(kEndEffectorRollerMotorConfig));
    }

    private Command setRollerMotor(double power) {
        return runOnce(() -> rollerMotor.set(power));
    }

    public Command scoreCoral(Supplier<ReefLevel> reefLevelSup) {
        return Commands.either(placeCoralInTrough(), placeCoralOnBranch(), () -> reefLevelSup.get().equals(ReefLevel.L1));
    }

    private Command placeCoralOnBranch() {
        return Commands.sequence(
            setRollerMotor(kCoralOuttakePower),
            Commands.waitSeconds(kCoralOuttakeWait),
            setRollerMotor(0.0)
        );
    }

    private Command placeCoralInTrough() {
        return Commands.sequence(
            setRollerMotor(kCoralOuttakeToTrough),
            Commands.waitSeconds(kCoralOuttakeWaitToTrough),
            setRollerMotor(0.0)
        );
    }

    public Command pickupCoralFromCradle() { 
        return Commands.sequence(
            setRollerMotor(kReceiveFromCradlePower),
            Commands.waitUntil(this::hasCoral),
            setRollerMotor(kHoldCoralPower)
        );
    }

    public Command intakeAlgae() {
        return Commands.sequence(
            setRollerMotor(kIntakeAlgaePower),
            Commands.waitUntil(this::hasAlgae),
            setRollerMotor(kHoldAlgaePower)
        );
    }

    public Command scoreAlgae(Supplier<AlgaeScorePath> algaeScorePathSup) {
        return new SelectCommand<AlgaeScorePath>(
            Map.ofEntries(
                Map.entry(AlgaeScorePath.NONE, Commands.none()),
                Map.entry(AlgaeScorePath.TO_NET, outtakeNetAlgae()),
                Map.entry(AlgaeScorePath.TO_PROCESSOR, outtakeProcessorAlgae())
            ),
            algaeScorePathSup
        ); 
    }

    public Command outtakeProcessorAlgae() {
        return Commands.sequence(
            setRollerMotor(kProcessorOuttakePower), 
            Commands.waitSeconds(kAlgaeOuttakeWait),
            setRollerMotor(0.0)
        );
    }

    public Command outtakeNetAlgae() {
        return Commands.sequence(
            setRollerMotor(kNetOuttakePower),
            Commands.waitSeconds(kAlgaeOuttakeWait),
            setRollerMotor(0.0)
        );
    }

    public Command stopRollers() {
        return setRollerMotor(0.0);
    }

    public boolean hasAlgae() {
        return rollerCurrentSignal.getValueAsDouble() > kHasAlgaeCurrent;
    }

    public boolean hasCoral() {
        return rollerCurrentSignal.getValueAsDouble() > kHasCoralCurrent && rollerCurrentSignal.getValueAsDouble() < kHasAlgaeCurrent;
    }

    @Override
    public void periodic() {
        rollerCurrentSignal.refresh();

        SmartDashboard.putBoolean("Has Algae", hasAlgae());
        SmartDashboard.putBoolean("End Effector Has Coral", hasCoral());
    }
}