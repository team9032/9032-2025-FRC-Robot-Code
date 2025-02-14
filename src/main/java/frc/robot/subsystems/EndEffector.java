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

    private final DigitalInput photoelectricSensor1 = new DigitalInput(kPhotoelectricSensor1ID); //TODO not a thing yet
    private final DigitalInput photoelectricSensor2 = new DigitalInput(kPhotoelectricSensor2ID); //TODO not a thing yet
    private final TimeOfFlight algaeDistSensor = new TimeOfFlight(kAlgaeDistSensorID); //TODO not a thing yet

    public EndEffector() {
        endEffectorMainMotor = new TalonFX(kMainEndEffectorID);
        endEffectorSecondaryMotor = new TalonFX(kSecondaryEndEffectorID);

        endEffectorMainMotor.optimizeBusUtilization();
        endEffectorSecondaryMotor.optimizeBusUtilization();
        
        ElasticUtil.checkStatus(endEffectorMainMotor.getConfigurator().apply(kEndEffectorConfig));
        ElasticUtil.checkStatus(endEffectorSecondaryMotor.getConfigurator().apply(kEndEffectorConfig));
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

    public Command pickupCoralFromSource() {
        return Commands.sequence(
            setEndEffectorMotors(kIntakeFromSourcePower),
            Commands.waitUntil(() -> photoelectricSensor1.get()),//TODO this does not match the current design!
            setEndEffectorMotors(kSlowIntakeFromSourcePower),
            Commands.waitUntil(() -> photoelectricSensor2.get()),
            setEndEffectorMotors(0.0)
        );
    }

    public Command receiveCoralFromIndexer() {
        return Commands.sequence(
            setEndEffectorMotors(kReceiveFromIndexerPower),
            Commands.waitUntil(() -> photoelectricSensor2.get()),
            setEndEffectorMotors(kSlowReceiveFromIndexerPower),
            Commands.waitUntil(() -> photoelectricSensor1.get()),
            setEndEffectorMotors(0.0)
        );
    }

    public Command pickupAlgae() {
        return Commands.sequence(
            setEndEffectorMainMotor(kIntakeAlgaePower),
            setEndEffectorSecondaryMotor(-kIntakeAlgaePower), //TODO change if bad
            Commands.waitUntil(this::hasAlgae), 
            setEndEffectorMotors(0.0)
        );
    }

    public Command outtakeProcessorAlgae() {
        return Commands.sequence(
            setEndEffectorMainMotor(kProcessorOuttakePower), //TODO change if bad
            setEndEffectorSecondaryMotor(-kProcessorOuttakePower),
            Commands.waitSeconds(kOuttakeWait),
            setEndEffectorMotors(0.0)
        );
    }

    public Command outtakeNetAlgae() {
        return Commands.sequence(
            setEndEffectorMainMotor(kNetOuttakePower), //TODO change if bad
            setEndEffectorSecondaryMotor(-kNetOuttakePower),
            Commands.waitSeconds(kOuttakeWait),
            setEndEffectorMotors(0.0)
        );
    }



    public boolean hasAlgae() {
        return algaeDistSensor.getRange() < kHasAlgaeDist;
    }

    public boolean hasCoral() {
        return photoelectricSensor1.get() && photoelectricSensor2.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("End Effector Photoelectric Sensor (first from source)", photoelectricSensor1.get());
        SmartDashboard.putBoolean("End Effector Photoelectric Sensor (first from indexer)", photoelectricSensor2.get());
        SmartDashboard.putNumber("Algae Sensor Dist", algaeDistSensor.getRange());
        SmartDashboard.putBoolean("Has Algae", hasAlgae());
        SmartDashboard.putBoolean("Has Coral", hasCoral());
    }
}