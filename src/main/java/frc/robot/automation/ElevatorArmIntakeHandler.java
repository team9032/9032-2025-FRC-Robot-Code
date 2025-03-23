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

    public Command moveToStowPosition() {
        return Commands.sequence(
            arm.moveToStowPos(),
            elevator.moveToOverIndexerPosition(),
            intake.returnToStowPosition(),
            Commands.waitUntil(() -> arm.atSetpoint() && elevator.atSetpoint()),
            ElasticUtil.sendInfoCommand("Moved to stow position")
        );
    }

    public Command prepareForCoralIntaking() {
        return Commands.sequence(
            intake.moveToEndEffectorMovePosition(),
            Commands.waitUntil(intake::endEffectorCanMovePast),
            arm.moveToIndexerPos(),
            elevator.moveToOverIndexerPosition(),
            Commands.waitUntil(() -> arm.atSetpoint() && elevator.atSetpoint()),
            elevator.moveToIndexerPosition(),
            Commands.waitUntil(elevator::atSetpoint),
            intake.returnToStowPosition(),
            ElasticUtil.sendInfoCommand("Prepared for coral intaking")
        );
    }

    public Command prepareForCoralScoringInitial() {
        return Commands.sequence(
            elevator.moveToOverIndexerPosition(),
            intake.moveToEndEffectorMovePosition(),
            Commands.waitUntil(() -> intake.endEffectorCanMovePast() && elevator.atSetpoint()),
            arm.moveToStowPos(),
            Commands.waitUntil(arm::atSetpoint),
            intake.returnToStowPosition(),
            ElasticUtil.sendInfoCommand("Prepared for coral scoring initial")
        );
    }

    public Command prepareForCoralScoringFinal() {
        return Commands.sequence(
            prepareForCoralScoringInitial()
                .onlyIf(() -> !elevator.overIndexPosition()),
            buttonBoardHandler.moveElevatorToCoralTargetLevel(elevator),
            buttonBoardHandler.moveArmToCoralTargetLevel(arm),
            Commands.waitUntil(() -> arm.atSetpoint() && elevator.atSetpoint()),
            ElasticUtil.sendInfoCommand("Prepared for coral scoring final")
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
}
