package frc.robot.automation;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.automation.ButtonBoardHandler.AlgaeScorePath;
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

    public Command moveToIntakePosition() {
        return Commands.either(
            Commands.sequence(
                    /* Don't deep climb the reef */
                    elevator.moveToCoralScoreLevel(() -> ReefLevel.L4)
                    .andThen(
                        Commands.waitUntil(elevator::atSetpoint),
                        arm.moveToStowPos(),
                        Commands.waitUntil(arm::atSetpoint)
                    )
                    .onlyIf(() -> arm.closeToL4() && elevator.overHighAlgae()),
                arm.moveToStowPos(),
                elevator.moveToOverIndexerPosition(),
                intake.moveToGround(),
                Commands.waitUntil(() -> intake.endEffectorCanMovePast() && elevator.overIndexPosition()),
                arm.moveToIndexerPos(),
                Commands.waitUntil(arm::atSetpoint),
                elevator.moveToIndexerPosition(),
                Commands.waitUntil(() -> elevator.atSetpoint() && intake.canRunRollers()),
                ElasticUtil.sendInfoCommand("Moved to index position")
            ),
            Commands.sequence(
                elevator.moveToIndexerPosition(),
                arm.moveToIndexerPos(),
                intake.moveToGround(),
                Commands.waitUntil(() -> elevator.atSetpoint() && arm.atSetpoint()),
                ElasticUtil.sendInfoCommand("Moved to index position from index position")
            ),
            () -> !arm.closeToIndexPosition()
        );  
    }

    public Command moveToStowPositions() {
        return Commands.either(
            Commands.sequence(
                elevator.moveToOverIndexerPosition(),
                intake.moveToGround(),
                Commands.waitUntil(() -> intake.endEffectorCanMovePast() && elevator.atSetpoint()),
                arm.moveToStowPos(),
                Commands.waitUntil(arm::overIntake),
                elevator.moveToStowPosition(),
                Commands.waitUntil(() -> arm.atSetpoint() && elevator.atSetpoint()),
                ElasticUtil.sendInfoCommand("Moved to stow from index")
            ),
            Commands.sequence(
                    /* Don't deep climb the reef */
                    elevator.moveToCoralScoreLevel(() -> ReefLevel.L4)
                        .andThen(
                            Commands.waitUntil(elevator::atSetpoint),
                            arm.moveToStowPos(),
                            Commands.waitUntil(arm::atSetpoint)
                        )
                    .onlyIf(() -> arm.closeToL4() && elevator.overHighAlgae()),
                arm.moveToStowPos(),
                elevator.moveToStowPosition(),
                Commands.waitUntil(() -> arm.atSetpoint() && elevator.atSetpoint()),
                ElasticUtil.sendInfoCommand("Moved to stow")
            ),
            arm::closeToIndexPosition
        );
    }

    public Command prepareForCoralScoring(Supplier<ReefLevel> reefLevelSup) {
        return Commands.sequence(
            moveToStowPositions()
                .onlyIf(() -> !reefLevelSup.get().equals(ReefLevel.L1)),
            elevator.moveToCoralScoreLevel(reefLevelSup),
            Commands.waitUntil(elevator::atSetpoint),
            arm.moveToCoralScoreLevel(reefLevelSup),
            Commands.waitUntil(arm::atSetpoint),
            ElasticUtil.sendInfoCommand("Prepared for coral scoring")
        );
    }

    public Command prepareForAutoCoralScoring(BooleanSupplier moveElevatorTrigger) {//TODO remove this method
        return Commands.sequence(
            moveToStowPositions(),
            Commands.waitUntil(moveElevatorTrigger),
            elevator.moveToCoralScoreLevel(() -> ReefLevel.L4),
            Commands.waitUntil(elevator::atSetpoint),
            arm.moveToCoralScoreLevel(() -> ReefLevel.L4),
            Commands.waitUntil(arm::atSetpoint)
        );
    }

    public Command prepareForAlgaeReefIntaking(BooleanSupplier isLowAlgaeSup) {
        return Commands.sequence(
            moveToStowPositions()
                .onlyIf(arm::closeToIndexPosition),
            Commands.either(
                elevator.moveToLowAlgaePosition()
                    .andThen(arm.moveToLowAlgaePos()), 
                elevator.moveToHighAlgaePosition()
                    .andThen(arm.moveToHighAlgaePos()), 
                isLowAlgaeSup
            ),
            Commands.waitUntil(this::elevatorAndArmAtSetpoints),
            ElasticUtil.sendInfoCommand("Prepared for algae reef intaking")
        );          
    }

    public Command prepareForAlgaeGroundIntaking() {
        return Commands.sequence(
            moveToStowPositions()
                .onlyIf(arm::closeToIndexPosition),
            arm.moveToAlgaeGroundPos(),
            elevator.moveToAlgaeGroundPosition(),
            Commands.waitUntil(this::elevatorAndArmAtSetpoints),
            ElasticUtil.sendInfoCommand("Prepared for algae ground intaking")
        );          
    }

    public Command prepareForAlgaeScoring(Supplier<AlgaeScorePath> algaeScorePathSup) {
        return Commands.sequence(
            Commands.either(
                elevator.moveToNetPosition()
                    .andThen(
                        Commands.waitUntil(elevator::closeToNetPosition),
                        arm.moveToNetPos()
                    ),
                elevator.moveToProcessorPosition()
                    .andThen(arm.moveToProcessorPos()),
                () -> algaeScorePathSup.get().equals(AlgaeScorePath.TO_NET)
            ),
            Commands.waitUntil(arm::atSetpoint),
            ElasticUtil.sendInfoCommand("Prepared for algae scoring")
        );
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
            return arm.atTrough() && elevator.atTrough();

        else if (reefLevel.equals(ReefLevel.L2))
            return arm.atL1() && elevator.atL1();

        else if (reefLevel.equals(ReefLevel.L3))
            return arm.atL2() && elevator.atL2();

        else if (reefLevel.equals(ReefLevel.L4))
            return arm.atL3() && elevator.atL3();
        
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
