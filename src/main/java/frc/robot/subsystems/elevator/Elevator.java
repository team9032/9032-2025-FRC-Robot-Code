package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import frc.robot.automation.ButtonBoardHandler.ReefLevel;
import frc.robot.subsystems.elevator.Elevator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import java.util.Map;
import java.util.function.Supplier;

public class Elevator extends SubsystemBase {
    private final MotionMagicExpoVoltage motionMagic = new MotionMagicExpoVoltage(0);

    private final ElevatorIO io;

    private final Mechanism2d visualizer;
    private final MechanismLigament2d ligament;

    public Elevator() {
        if (RobotBase.isSimulation())
            io = new ElevatorIOSim();

        else 
            io = new ElevatorIOReal();

        visualizer = new Mechanism2d(3.0, 3.0);
        ligament = new MechanismLigament2d("1st stage", 0.0, 90.0);
        visualizer.getRoot("Elevator", 1.3, 0.0)
            .append(ligament);

        SmartDashboard.putData("Elevator visualizer", visualizer);
    }

    private void moveElevator(double pos) {
        io.setControl(motionMagic.withPosition(pos));
    }

    private boolean atPosition(double position) {
        return MathUtil.isNear(position, io.getPosition(), kElevatorTolerance);
    }

    public boolean atL1() {
        return atPosition(kElevatorL1);
    }

    public boolean atL2() {
        return atPosition(kElevatorL2);
    }

    public boolean atL3() {
        return atPosition(kElevatorL3);
    }

    public boolean atL4() {
        return atPosition(kElevatorL4);
    }
    
    public boolean atSetpoint() {
        return atPosition(motionMagic.Position);
    }

    public boolean overCradlePosition() {
        return io.getPosition() > (kElevatorOverCradle - kElevatorTolerance);
    }

    public boolean closeToNetPosition() {
        return io.getPosition() > kElevatorCloseToNet;
    }

    public Command holdPosition() {
        return runOnce(() -> moveElevator(io.getPosition()));
    }

    public Command moveToStowPosition() {
        return runOnce(() -> moveElevator(kElevatorStow));
    }

    public Command moveToCradlePosition() {
        return runOnce(() -> moveElevator(kElevatorCradlePos));
    }

    public Command moveToLowAlgaePosition() {
        return runOnce(() -> moveElevator(kElevatorLowAlgae));
    }

    public Command moveToHighAlgaePosition() {
        return runOnce(() -> moveElevator(kElevatorHighAlgae));
    }

    public Command moveToClimbPosition() {
        return runOnce(() -> moveElevator(kElevatorClimb));
    }

    public Command moveToCoralScoreLevel(Supplier<ReefLevel> reefLevelSup) {
        return new SelectCommand<ReefLevel>(
            Map.ofEntries(
                Map.entry(ReefLevel.NONE, Commands.none()),
                Map.entry(ReefLevel.L1, runOnce(() -> moveElevator(kElevatorL1))),
                Map.entry(ReefLevel.L2, runOnce(() -> moveElevator(kElevatorL2))),
                Map.entry(ReefLevel.L3, runOnce(() -> moveElevator(kElevatorL3))),
                Map.entry(ReefLevel.L4, runOnce(() -> moveElevator(kElevatorL4)))
            ),
            reefLevelSup
        );
    }

    public Command moveToProcessorPosition() {
        return runOnce(() -> moveElevator(kElevatorProcessor));
    }

    public Command moveToNetPosition() {
        return runOnce(() -> moveElevator(kElevatorNet));
    }

    public Command moveToOverCradlePosition() {
        return runOnce(() -> moveElevator(kElevatorOverCradle));
    }

    public Command moveToAlgaeGroundPosition() {
        return runOnce(() -> moveElevator(kElevatorAlgaeGround));
    }

    public boolean overHighAlgae() {
        return io.getPosition() > kElevatorOverHighAlgae;
    }

    public Command coast() {
        return runOnce(() -> io.setControl(new CoastOut()))
            .ignoringDisable(true);
    }

    @Override
    public void periodic() {
        io.periodic();

        SmartDashboard.putBoolean("Elevator At Setpoint", atSetpoint());

        ligament.setLength(io.getPosition() + 0.25);
    }
}