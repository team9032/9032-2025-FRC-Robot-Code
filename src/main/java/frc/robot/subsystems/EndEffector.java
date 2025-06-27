package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.Status;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.automation.ButtonBoardHandler.AlgaeScorePath;
import frc.robot.automation.ButtonBoardHandler.ReefLevel;
import frc.robot.utils.ElasticUtil;

import static frc.robot.Constants.EndEffectorConstants.*;

import java.util.Map;
import java.util.function.Supplier;

public class EndEffector extends SubsystemBase {
    private final TalonFX endEffectorMainMotor;
    private final TalonFX endEffectorSecondaryMotor;

    private final DigitalInput indexerPhotoelectricSensor = new DigitalInput(kIndexerPhotoelectricSensorID); 
    private final DigitalInput sourcePhotoelectricSensor = new DigitalInput(kSourcePhotoelectricSensorID); 
    private final TimeOfFlight algaeDistSensor = new TimeOfFlight(kAlgaeDistSensorID); 

    private boolean lastHasAlgae = false;
    private int loopCyclesSinceTOFReading = 0;

    public EndEffector() {
        endEffectorMainMotor = new TalonFX(kMainEndEffectorID);
        endEffectorSecondaryMotor = new TalonFX(kSecondaryEndEffectorID);

        endEffectorMainMotor.optimizeBusUtilization();
        endEffectorSecondaryMotor.optimizeBusUtilization();
        
        ElasticUtil.checkStatus(endEffectorMainMotor.getConfigurator().apply(kMainEndEffectorConfig));
        ElasticUtil.checkStatus(endEffectorSecondaryMotor.getConfigurator().apply(kSecondaryEndEffectorConfig));

        algaeDistSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 50);
        algaeDistSensor.setRangeOfInterest(8, 8, 12, 12);
    }

    private Command setEndEffectorMainMotor(double power) {
        return runOnce(() -> endEffectorMainMotor.set(power));
    }

    private Command setEndEffectorSecondaryMotor(double power) {
        return runOnce(() -> endEffectorSecondaryMotor.set(power));
    }

    private void setEndEffectorMotorsAlgae(double power) {
        endEffectorMainMotor.set(power);
        endEffectorSecondaryMotor.set(-power);
    }

    private void setEndEffectorMotors(double power) {
        endEffectorMainMotor.set(power);
        endEffectorSecondaryMotor.set(power);
    }

    private Command setEndEffectorMotorsCommand(double power) {
        return runOnce(() -> setEndEffectorMotors(power));
    }

    public Command scoreCoral(Supplier<ReefLevel> reefLevelSup) {
        return Commands.either(placeCoralInTrough(), placeCoral(), () -> reefLevelSup.get().equals(ReefLevel.L1));
    }

    private Command placeCoral() {
        return Commands.sequence(
            setEndEffectorMotorsCommand(kCoralOuttakePower),
            Commands.waitSeconds(kCoralOuttakeWait),
            setEndEffectorMotorsCommand(0.0)
        );
    }

    private Command placeCoralInTrough() {
        return Commands.sequence(
            setEndEffectorMotorsCommand(kCoralOuttakeToTrough),
            Commands.waitSeconds(kCoralOuttakeWaitToTrough),
            setEndEffectorMotorsCommand(0.0)
        );
    }

    public Command pickupCoralFromSource() {
        return Commands.sequence(
            setEndEffectorMotorsCommand(kIntakeFromSourcePower),
            Commands.waitUntil(() -> getSourcePhotoelectricSensor()),
            setEndEffectorMotorsCommand(kSlowIntakeFromSourcePower),
            Commands.waitUntil(() -> getIndexerPhotoelectricSensor()),
            setEndEffectorMotorsCommand(0.0)
        );
    }

    public Command holdCoral() { 
        return run(() -> {
            if (!getSourcePhotoelectricSensor()) {
                setEndEffectorMotors(kSlowReceiveFromIndexerPower);
            }

            else if (!getIndexerPhotoelectricSensor())
                setEndEffectorMotors(kSlowIntakeFromSourcePower);
            
            else if (hasCoralCentered())
                setEndEffectorMotors(0.0);
        });
    }

    public Command holdAlgae() {
        return run(() -> {
            if (hasAlgaeNearby() && !hasAlgae())
                setEndEffectorMotorsAlgae(kIntakeAlgaeSlowPower);

            else    
                setEndEffectorMotorsAlgae(kHoldAlgaePower);
        });
    }

    public Command receiveCoralFromIndexer() {
        return Commands.sequence(
            setEndEffectorMotorsCommand(kReceiveFromIndexerPower),
            Commands.waitUntil(() -> getIndexerPhotoelectricSensor()),
            setEndEffectorMotorsCommand(kSlowReceiveFromIndexerPower),
            Commands.waitUntil(() -> getSourcePhotoelectricSensor()),
            setEndEffectorMotorsCommand(0.0)
        );
    }

    public Command pickupAlgae() {
        return Commands.sequence(
            setEndEffectorMainMotor(kIntakeAlgaePower),
            setEndEffectorSecondaryMotor(-kIntakeAlgaePower), 
            Commands.waitUntil(this::hasAlgaeNearby),
            new ScheduleCommand(holdAlgae())
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
            setEndEffectorMainMotor(kProcessorOuttakePower), 
            setEndEffectorSecondaryMotor(-kProcessorOuttakePower),
            Commands.waitSeconds(kAlgaeOuttakeWait),
            setEndEffectorMotorsCommand(0.0)
        );
    }

    public Command outtakeNetAlgae() {
        return Commands.sequence(
            setEndEffectorMainMotor(kNetOuttakePower),
            setEndEffectorSecondaryMotor(-kNetOuttakePower),
            Commands.waitSeconds(kAlgaeOuttakeWait),
            setEndEffectorMotorsCommand(0.0)
        );
    }

    public Command stopRollers() {
        return setEndEffectorMotorsCommand(0.0);
    }

    public boolean hasAlgae() {
        return lastHasAlgae;
    }

    public boolean hasAlgaeNearby() {
        if (algaeDistSensor.getStatus().equals(Status.Valid)) {
            boolean hasAlgae = algaeDistSensor.getRange() < kHasAlgaeNearbyDist && !hasCoral() && algaeDistSensor.getRange() != 0;

            return hasAlgae;
        }

        return false;
    }

    public boolean hasCoralCentered() {
        return getIndexerPhotoelectricSensor() && getSourcePhotoelectricSensor();
    }

    public boolean hasCoral() {
        return getSourcePhotoelectricSensor() || getIndexerPhotoelectricSensor();
    }

    private boolean getSourcePhotoelectricSensor() {
        return !sourcePhotoelectricSensor.get();
    }

    private boolean getIndexerPhotoelectricSensor() {
        return !indexerPhotoelectricSensor.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("End Effector Photoelectric Sensor (first from indexer)", getIndexerPhotoelectricSensor());
        SmartDashboard.putBoolean("End Effector Photoelectric Sensor (first from source)", getSourcePhotoelectricSensor());
        SmartDashboard.putNumber("Algae Sensor Dist", algaeDistSensor.getRange());
        SmartDashboard.putBoolean("Has Algae", hasAlgae());
        SmartDashboard.putBoolean("End Effector Has Coral", hasCoral());

        if (algaeDistSensor.getStatus().equals(Status.Valid)) {
            boolean hasAlgae = algaeDistSensor.getRange() < kHasAlgaeDist && !hasCoral() && algaeDistSensor.getRange() != 0;

            lastHasAlgae = hasAlgae;
            loopCyclesSinceTOFReading = 0;
        }

        else {
            loopCyclesSinceTOFReading++;

            if (loopCyclesSinceTOFReading > kCycleAmountForTOFToExpire) {
                loopCyclesSinceTOFReading = 0;

                lastHasAlgae = false;
            }
        }
    }
}