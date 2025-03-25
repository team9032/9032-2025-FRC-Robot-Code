package frc.robot.automation;

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

    public Command moveToIntakePosition(boolean intakeDown) {
        return Commands.either(
            Commands.sequence(
                elevator.moveToOverIndexerPosition(),
                Commands.either(
                    intake.moveToGround(),
                    intake.moveToEndEffectorMovePosition(),
                    () -> intakeDown
                ),
                Commands.waitUntil(() -> intake.endEffectorCanMovePast() && elevator.atSetpoint()),
                arm.moveToIndexerPos(),
                Commands.waitUntil(arm::atSetpoint),
                elevator.moveToIndexerPosition(),
                Commands.waitUntil(elevator::atSetpoint),
                Commands.either(
                    Commands.waitUntil(intake::canRunRollers),
                    intake.returnToStowPosition(),
                    () -> intakeDown
                ),
                ElasticUtil.sendInfoCommand("Moved to index position - intake down is " + intakeDown)
            ),
            Commands.sequence(
                elevator.moveToIndexerPosition(),
                arm.moveToIndexerPos(),
                intake.moveToGround()
                    .onlyIf(() -> intakeDown),
                Commands.waitUntil(() -> elevator.atSetpoint() && arm.atSetpoint()),
                ElasticUtil.sendInfoCommand("Moved to index position from index position - intake down is " + intakeDown)
            ),
            () -> !arm.closeToIndexPosition()
        );  
    }

    public Command moveToStowPositions() {
        return Commands.either(
            Commands.sequence(
                elevator.moveToOverIndexerPosition(),
                intake.moveToEndEffectorMovePosition(),
                Commands.waitUntil(() -> intake.endEffectorCanMovePast() && elevator.atSetpoint()),
                arm.moveToStowPos(),
                Commands.waitUntil(arm::atSetpoint),
                intake.returnToStowPosition(),
                ElasticUtil.sendInfoCommand("Moved to stow from index")
            ),
            Commands.sequence(
                elevator.moveToOverIndexerPosition(),
                intake.returnToStowPosition(),
                arm.moveToStowPos(),
                Commands.waitUntil(() -> arm.atSetpoint() && elevator.atSetpoint()),
                ElasticUtil.sendInfoCommand("Moved to stow")
            ),
            arm::closeToIndexPosition
        );
    }

    public Command prepareForCoralScoring() {
        return Commands.sequence(
            moveToStowPositions(),
            buttonBoardHandler.moveElevatorToCoralTargetLevel(elevator),
            Commands.waitUntil(elevator::atSetpoint),
            buttonBoardHandler.moveArmToCoralTargetLevel(arm),
            Commands.waitUntil(arm::atSetpoint),
            ElasticUtil.sendInfoCommand("Prepared for coral scoring")
        );
    }

    public Command prepareForAlgaeIntakingInitial() {
        return Commands.sequence(
            elevator.moveToOverIndexerPosition(),
            Commands.waitUntil(elevator::atSetpoint),
            buttonBoardHandler.moveArmToAlgaeIntakeTargetLevel(arm),
            Commands.waitUntil(arm::atSetpoint),
            ElasticUtil.sendInfoCommand("Prepared for algae intaking inital")
        );
    }

    public Command prepareForAlgaeIntakingFinal() {
        return Commands.sequence(
            prepareForAlgaeIntakingInitial()
                .onlyIf(() -> !elevator.overIndexPosition()),
            buttonBoardHandler.moveElevatorToAlgaeIntakeTargetLevel(elevator),
            buttonBoardHandler.moveArmToAlgaeIntakeTargetLevel(arm),
            Commands.waitUntil(() -> arm.atSetpoint() && elevator.atSetpoint()),
            ElasticUtil.sendInfoCommand("Prepared for algae intaking final")
        );
    }

    public Command prepareForAlgaeScoring() {
        return Commands.sequence(
            buttonBoardHandler.moveElevatorToAlgaeScoreLevel(elevator),
            buttonBoardHandler.moveArmToAlgaeScoreLevel(arm),
            Commands.waitUntil(() -> arm.atSetpoint() && elevator.atSetpoint()),
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
}
