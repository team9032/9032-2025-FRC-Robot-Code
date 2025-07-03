package frc.robot.automation;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.automation.ButtonBoardHandler.ReefLevel;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.utils.ElasticUtil;

public class ElevatorArmIntakeHandler {
    private final Elevator elevator;
    private final Arm arm;
    private final Intake intake;

    public ElevatorArmIntakeHandler(Elevator elevator, Arm arm, Intake intake) {
        this.elevator = elevator; 
        this.arm = arm;
        this.intake = intake;
    }

    public Command moveIntakeUp() {
        return intake.returnToStowPosition();
    }

    public Command moveIntakeDown() {
        return intake.moveToGround();
    }

    public Command moveToCoralCradlePosition() {
        return elevator.moveToCradlePosition();
    }

    private Command moveOutOfCradleIfNeeded() {
        return Commands.sequence(
            arm.holdPosition(),
            elevator.moveToOverCradlePosition(),
            Commands.waitUntil(elevator::atSetpoint)
        ).onlyIf(() -> !arm.overCradle());
    }       

    public Command moveToIntakePosition() {
        return Commands.sequence(
            moveOutOfCradleIfNeeded(),
            arm.moveToCradlePos(),
            elevator.moveToOverCradlePosition(),
            Commands.waitUntil(this::elevatorAndArmAtSetpoints),
            ElasticUtil.sendInfoCommand("Moved to intake position")
        ); 
    }

    public Command moveToStowPositions() {
        return Commands.sequence(
            moveOutOfCradleIfNeeded(),
            arm.moveToStowPos(),
            Commands.waitUntil(arm::overCradle),
            elevator.moveToStowPosition(),
            Commands.waitUntil(this::elevatorAndArmAtSetpoints),
            ElasticUtil.sendInfoCommand("Moved to stow")
        );
    }

    public Command moveToStowPositionsFromNet() {
        return Commands.sequence(
            arm.moveToStowPos(),
            Commands.waitUntil(arm::atSetpoint),
            elevator.moveToStowPosition(),
            Commands.waitUntil(elevator::atSetpoint),
            ElasticUtil.sendInfoCommand("Moved to stow from net")
        );
    }

    public Command prepareForCoralScoring(Supplier<ReefLevel> reefLevelSup) {
        return Commands.sequence(
            moveOutOfCradleIfNeeded(),
            elevator.moveToCoralScoreLevel(reefLevelSup),
            Commands.waitUntil(elevator::atSetpoint)
                .onlyIf(() -> reefLevelSup.get().equals(ReefLevel.L4)),
            arm.moveToPreparedToScoreCoralPos(),
            Commands.waitUntil(this::elevatorAndArmAtSetpoints),
            ElasticUtil.sendInfoCommand("Prepared for coral scoring")
        );
    }

    public Command prepareForAlgaeReefIntaking(BooleanSupplier isHighAlgaeSup) {
        return Commands.sequence(
            moveOutOfCradleIfNeeded(),
            Commands.either(
                elevator.moveToHighAlgaePosition()
                    .andThen(arm.moveToHighAlgaePos()), 
                elevator.moveToLowAlgaePosition()
                    .andThen(arm.moveToLowAlgaePos()), 
                isHighAlgaeSup
            ),
            Commands.waitUntil(this::elevatorAndArmAtSetpoints),
            ElasticUtil.sendInfoCommand("Prepared for algae reef intaking")
        );          
    }

    public Command prepareForAlgaeGroundIntaking() {
        return Commands.sequence(
            moveOutOfCradleIfNeeded(),
            arm.moveToAlgaeGroundPos(),
            elevator.moveToAlgaeGroundPosition(),
            Commands.waitUntil(this::elevatorAndArmAtSetpoints),
            ElasticUtil.sendInfoCommand("Prepared for algae ground intaking")
        );          
    }

    public Command prepareForNetAlgaeScoring() {
        return Commands.sequence(
            moveOutOfCradleIfNeeded(),
            arm.moveToStowPos(),
            Commands.waitUntil(arm::atSetpoint),
            elevator.moveToNetPosition(),
            Commands.waitUntil(elevator::closeToNetPosition),
            arm.moveToNetPos(),
            Commands.waitUntil(arm::atSetpoint),
            ElasticUtil.sendInfoCommand("Prepared for net algae scoring")
        );
    }

    public Command prepareForProcessorAlgaeScoring() {
        return Commands.sequence(
            moveOutOfCradleIfNeeded(),
            elevator.moveToProcessorPosition(),
            arm.moveToProcessorPos(),
            Commands.waitUntil(this::elevatorAndArmAtSetpoints),
            ElasticUtil.sendInfoCommand("Prepared for processor algae scoring")
        );
    }

    public Command prepareForClimbing() {
        return Commands.sequence(
            moveOutOfCradleIfNeeded(),
            arm.moveToClimbPos(),
            elevator.moveToClimbPosition(),
            intake.returnToStowPosition(),
            Commands.waitUntil(() -> elevatorAndArmAtSetpoints() && intake.readyForClimbing())
        );
    }

    public Command moveArmToCoralScorePos(Supplier<ReefLevel> reefLevelSup) {
        return arm.moveToCoralScoreLevel(reefLevelSup)
            .andThen(Commands.waitUntil(arm::atSetpoint));
    }

    public Command holdPositions() {
        return Commands.sequence(
            arm.holdPosition(),
            elevator.holdPosition(),
            intake.holdPosition()
        );
    }

    public boolean elevatorAndArmAtSetpoints() {
        return elevator.atSetpoint() && arm.atSetpoint();
    }

    public boolean readyToScoreCoral(ReefLevel reefLevel) {
        if (reefLevel.equals(ReefLevel.L1))
            return arm.atL1() && elevator.atL1();

        else if (reefLevel.equals(ReefLevel.L2))
            return arm.atCoralPreparedToScorePos() && elevator.atL2();

        else if (reefLevel.equals(ReefLevel.L3))
            return arm.atCoralPreparedToScorePos() && elevator.atL3();

        else if (reefLevel.equals(ReefLevel.L4))
            return arm.atCoralPreparedToScorePos() && elevator.atL4();
        
        return false;
    }

    public Command coastAll() {
        return Commands.sequence(
            arm.coast(),
            elevator.coast(),
            intake.coast()  
        )
        .ignoringDisable(true);
    }
}
