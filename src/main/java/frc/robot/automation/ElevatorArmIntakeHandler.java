package frc.robot.automation;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.utils.ElasticUtil;

public class ElevatorArmIntakeHandler {
    private final Elevator elevator;
    private final Arm arm;
    private final Intake intake;

    private final ButtonBoardHandler buttonBoardHandler;

    public ElevatorArmIntakeHandler(Elevator elevator, Arm arm, Intake intake, ButtonBoardHandler buttonBoardHandler) {
        this.elevator = elevator; 
        this.arm = arm;
        this.intake = intake;

        this.buttonBoardHandler = buttonBoardHandler;
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
                    elevator.moveToL3Position()
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
                    elevator.moveToL3Position()
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

    public Command prepareForCoralScoring() {
        return Commands.sequence(
            moveToStowPositions()
                .onlyIf(buttonBoardHandler::l1NotSelected),
            buttonBoardHandler.moveElevatorToCoralTargetLevel(elevator),
            Commands.waitUntil(elevator::atSetpoint),
            buttonBoardHandler.moveArmToCoralTargetLevel(arm),
            Commands.waitUntil(arm::atSetpoint),
            ElasticUtil.sendInfoCommand("Prepared for coral scoring")
        );
    }

    public Command prepareForAutoCoralScoring(BooleanSupplier moveElevatorTrigger) {
        return Commands.sequence(
            moveToStowPositions(),
            Commands.waitUntil(moveElevatorTrigger),
            elevator.moveToL3Position(),
            Commands.waitUntil(elevator::atSetpoint),
            arm.moveToLevel3Pos(),
            Commands.waitUntil(arm::atSetpoint)
        );
    }

    public Command prepareForAlgaeReefIntaking() {
        return Commands.sequence(
            moveToStowPositions()
                .onlyIf(arm::closeToIndexPosition),
            buttonBoardHandler.moveArmToAlgaeIntakeTargetLevel(arm),
            buttonBoardHandler.moveElevatorToAlgaeIntakeTargetLevel(elevator),
            Commands.waitUntil(this::elevatorAndArmAtSetpoints),
            ElasticUtil.sendInfoCommand("Prepared for algae reef intaking")

        );          
    }

    public Command prepareForAlgaeReefIntakingAuto(boolean highAlgae) {
        return Commands.sequence(
            moveToStowPositions()
                .onlyIf(arm::closeToIndexPosition),
            Commands.either(
                arm.moveToHighAlgaePos() 
                    .andThen(elevator.moveToHighAlgaePosition()),
                arm.moveToLowAlgaePos() 
                    .andThen(elevator.moveToLowAlgaePosition()), 
                () -> highAlgae
            ),
            Commands.waitUntil(this::elevatorAndArmAtSetpoints),
            ElasticUtil.sendInfoCommand("Prepared for algae reef intaking in auto")
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

    public Command prepareForAlgaeScoring() {//TODO handle processor
        return Commands.sequence(
            buttonBoardHandler.moveElevatorToAlgaeScoreLevel(elevator),
            Commands.waitUntil(elevator::closeToNetPosition),
            buttonBoardHandler.moveArmToAlgaeScoreLevel(arm),
            Commands.waitUntil(arm::atSetpoint),
            ElasticUtil.sendInfoCommand("Prepared for algae scoring")
        );
    }

    public Command prepareForAlgaeScoringAuto() {
        return Commands.sequence(
            elevator.moveToNetPosition(),
            Commands.waitUntil(elevator::closeToNetPosition),
            arm.moveToNetPos(),
            Commands.waitUntil(arm::atSetpoint),
            ElasticUtil.sendInfoCommand("Prepared for algae scoring")
        );
    }

    public Command prepareForL1Scoring() {
        return Commands.sequence(
            elevator.moveToTroughPosition(),
            Commands.waitUntil(elevator::atSetpoint),
            arm.moveToTroughPos(),
            Commands.waitUntil(arm::atSetpoint),
            ElasticUtil.sendInfoCommand("Prepared for L1 scoring")
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

    public boolean readyForCoralScoring() {
        return buttonBoardHandler.readyToScoreCoral(arm, elevator);
    }
}
