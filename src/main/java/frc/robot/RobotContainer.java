// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.Elastic;
import frc.lib.Elastic.Notification;
import frc.robot.Constants.DriverConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.swerve.KrakenSwerve;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController driveController = new CommandXboxController(
            DriverConstants.kDriveControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(3);

    /* Drive Controller Buttons */
    private final Trigger zeroGyro = driveController.b();

    /* Operator Controller Buttons */
    private final Trigger elevatorL4Button = operatorController.a();
    private final Trigger elevatorL1Button = operatorController.b();
    private final Trigger elevatorL2Button = operatorController.x();
    private final Trigger elevatorL3Button = operatorController.y();
    // ...

    //...
    private final Trigger armTrough = driveController.povUp();
    private final Trigger armLevel1 = driveController.povRight();
    private final Trigger armLevel2 = driveController.povDown();
    private final Trigger armLevel3 = driveController.povLeft();
    /* Subsystems */

    private final Intake intake = new Intake();
    private final Arm arm = new Arm();
    private final KrakenSwerve krakenSwerve = new KrakenSwerve();
    private final Elevator elevator = new Elevator();
    private final Indexer indexer = new Indexer();
    private final Climber climber = new Climber();

    /* Dashboard */
    private final Notification elasticNotification = new Notification();

    /* Robot Mode Triggers */
    // ...

    /* Teleop Triggers */
    private final Trigger groundPTrigger = driveController.y();
    private final Trigger stowPTrigger = driveController.x();

    /* Auto Triggers */
    // ...

    /* State Triggers */
    // ...

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureButtonTriggers();
    }

    /** Use this method to define your button trigger->command mappings. */
    private void configureButtonTriggers() {
        /* Driver Controls */
        krakenSwerve.setDefaultCommand(
            new TeleopSwerve(
                krakenSwerve,
                driveController::getRightX,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX())
        );

        zeroGyro.onTrue(
            krakenSwerve.zeroGyro()
            .andThen(sendInfoNotification("Zeroed gyro"))
        );

        groundPTrigger.onTrue(
            intake.moveToGround()
            .andThen(sendInfoNotification("Ground Position works"))
        );

        stowPTrigger.onTrue(
            intake.returnToStowPosition()
            .andThen(sendInfoNotification("Stow Position works"))
        );

        armTrough.onTrue(arm.armToTroughPos());
        armLevel1.onTrue(arm.armToLevel1Pos());
        armLevel2.onTrue(arm.armToLevel2Pos());
        armLevel3.onTrue(arm.armToLevel3Pos());
        
        climber.setDefaultCommand(
            climber.setPower(() -> (driveController.getLeftTriggerAxis() - driveController.getRightTriggerAxis()))
        );
        /* Operator Controls */
        elevatorL4Button.onTrue(
                elevator.elevatorL4Command());
        elevatorL1Button.onTrue(
                elevator.elevatorL1Command());
        elevatorL2Button.onTrue(
                elevator.elevatorL2Command());
        elevatorL3Button.onTrue(
                elevator.elevatorL3Command());
        // ...
    }

    /** Use this to pass the autonomous command */
    public Command getAutonomousCommand() {
        return null;
    }

    private Command sendInfoNotification(String info) {
        return new InstantCommand(() -> Elastic.sendNotification(elasticNotification.withTitle(info)));
    }
}
