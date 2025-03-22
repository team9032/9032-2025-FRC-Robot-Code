package frc.robot.automation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.ElasticUtil;

public class ElevatorAndArmHandler {
    private final Elevator elevator;
    private final Arm arm;

    private final ButtonBoardHandler buttonBoardHandler;

    public ElevatorAndArmHandler(Elevator elevator, Arm arm, ButtonBoardHandler buttonBoardHandler) {
        this.elevator = elevator; 
        this.arm = arm;
        this.buttonBoardHandler = buttonBoardHandler;
    }

    public Command prepareForCoralIntaking() {
        return Commands.sequence(
            arm.moveToIndexerPos(),
            elevator.moveToOverIndexerPosition(),            
            Commands.waitUntil(() -> arm.atSetpoint() && elevator.atSetpoint()),
            elevator.moveToIndexerPosition(),
            Commands.waitUntil(elevator::atSetpoint),
            ElasticUtil.sendInfoCommand("Prepared for coral intaking")
        );
    }

    public Command prepareForCoralScoringInitial() {
        return Commands.sequence(
            elevator.moveToOverIndexerPosition(),
            Commands.waitUntil(elevator::atSetpoint),
            buttonBoardHandler.moveArmToCoralTargetLevel(arm),
            Commands.waitUntil(arm::atSetpoint),
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
