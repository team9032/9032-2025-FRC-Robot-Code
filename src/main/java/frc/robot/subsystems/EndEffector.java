package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ElasticUtil;

import static frc.robot.Constants.EndEffectorConstants.*;

public class EndEffector extends SubsystemBase {
    private final TalonFX endEffectorMainMotor;
    private final TalonFX endEffectorSecondaryMotor;

    private final DigitalInput indexerPhotoelectricSensor = new DigitalInput(kIndexerPhotoelectricSensorID); 
    private final DigitalInput sourcePhotoelectricSensor = new DigitalInput(kSourcePhotoelectricSensorID); 
    private final TimeOfFlight algaeDistSensor = new TimeOfFlight(kAlgaeDistSensorID); 

    public EndEffector() {
        endEffectorMainMotor = new TalonFX(kMainEndEffectorID);
        endEffectorSecondaryMotor = new TalonFX(kSecondaryEndEffectorID);

        endEffectorMainMotor.optimizeBusUtilization();
        endEffectorSecondaryMotor.optimizeBusUtilization();
        
        ElasticUtil.checkStatus(endEffectorMainMotor.getConfigurator().apply(kMainEndEffectorConfig));
        ElasticUtil.checkStatus(endEffectorSecondaryMotor.getConfigurator().apply(kSecondaryEndEffectorConfig));

        algaeDistSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 200);
        algaeDistSensor.setRangeOfInterest(8, 8, 12, 12);
    }

    private Command setEndEffectorMainMotor(double power) {
        return runOnce(() -> endEffectorMainMotor.set(power));
    }

    private Command setEndEffectorSecondaryMotor(double power) {
        return runOnce(() -> endEffectorSecondaryMotor.set(power));
    }

    private Command setEndEffectorMotors(double power) {
        return runOnce(() -> {
            endEffectorMainMotor.set(power);
            endEffectorSecondaryMotor.set(power);
        });
    }

    public Command placeCoral() {
        return Commands.sequence(
            setEndEffectorMotors(kCoralOuttakePower),
            Commands.waitSeconds(kOuttakeWait),
            setEndEffectorMotors(0.0)
        );
    }

    public Command placeCoralInTrough() {
        return Commands.sequence(
            setEndEffectorMainMotor(kCoralOuttakeToTrough),
            Commands.waitSeconds(kOuttakeWait),
            setEndEffectorMotors(0.0)
        );
    }

    public Command pickupCoralFromSource() {
        return Commands.sequence(
            setEndEffectorMotors(kIntakeFromSourcePower),
            Commands.waitUntil(() -> getSourcePhotoelectricSensor()),
            setEndEffectorMotors(kSlowIntakeFromSourcePower),
            Commands.waitUntil(() -> getIndexerPhotoelectricSensor()),
            setEndEffectorMotors(0.0)
        );
    }

    public Command receiveCoralFromIndexer() {
        return Commands.sequence(
            setEndEffectorMotors(kReceiveFromIndexerPower),
            Commands.waitUntil(() -> getIndexerPhotoelectricSensor()),
            setEndEffectorMotors(kSlowReceiveFromIndexerPower),
            Commands.waitUntil(() -> getSourcePhotoelectricSensor()),
            setEndEffectorMotors(0.0)
        );
    }

    public Command pickupAlgae() {
        return Commands.sequence(
            setEndEffectorMainMotor(kIntakeAlgaePower),
            setEndEffectorSecondaryMotor(-kIntakeAlgaePower), 
            Commands.waitUntil(this::hasAlgae), 
            setEndEffectorMainMotor(kHoldAlgaePower),
            setEndEffectorSecondaryMotor(-kHoldAlgaePower)
        );
    }

    public Command outtakeProcessorAlgae() {
        return Commands.sequence(
            setEndEffectorMainMotor(kProcessorOuttakePower), 
            setEndEffectorSecondaryMotor(-kProcessorOuttakePower),
            Commands.waitSeconds(kOuttakeWait),
            setEndEffectorMotors(0.0)
        );
    }

    public Command outtakeNetAlgae() {
        return Commands.sequence(
            setEndEffectorMainMotor(kNetOuttakePower),
            setEndEffectorSecondaryMotor(-kNetOuttakePower),
            Commands.waitSeconds(kOuttakeWait),
            setEndEffectorMotors(0.0)
        );
    }

    public boolean hasAlgae() {
        return algaeDistSensor.getRange() < kHasAlgaeDist && !hasCoral() && algaeDistSensor.getRange() != 0;
    }

    public boolean hasCoral() {
        return getIndexerPhotoelectricSensor() && getSourcePhotoelectricSensor();
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
        SmartDashboard.putBoolean("Has Coral", hasCoral());
    }
}