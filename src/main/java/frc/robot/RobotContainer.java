// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.KrakenSwerve;
import frc.robot.utils.ElasticUtil;
import frc.robot.utils.GitData;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static frc.robot.Constants.DriverConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController driveController = new CommandXboxController(kDriveControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(3);

    /* Drive Controller Buttons */
    private final Trigger zeroGyro = driveController.b();
    private final Trigger scoreCoral = driveController.a();
    private final Trigger pickupCoral = driveController.y();
    private final Trigger groundPTrigger = driveController.y();
    private final Trigger stowPTrigger = driveController.x();
    private final Trigger armTrough = driveController.povUp();
    private final Trigger armLevel1 = driveController.povRight();
    private final Trigger armLevel2 = driveController.povDown();
    private final Trigger armLevel3 = driveController.povLeft();

    /* Operator Controller Buttons */
    private final Trigger elevatorL3Button = operatorController.a();
    private final Trigger elevatorTroughButton = operatorController.b();
    private final Trigger elevatorL1Button = operatorController.x();
    private final Trigger elevatorL2Button = operatorController.y();
    private final Trigger index = operatorController.leftTrigger();
    private final Trigger eject = operatorController.rightTrigger();

    /* Subsystems */
    private final Intake intake = new Intake();
    private final Arm arm = new Arm();
    private final KrakenSwerve krakenSwerve = new KrakenSwerve();
    private final Elevator elevator = new Elevator();
    private final Indexer indexer = new Indexer();
    private final Climber climber = new Climber();
    private final EndEffector endEffector = new EndEffector();

    /* Dashboard */
    private final SendableChooser<Command> autoChooser;

    /* Robot Mode Triggers */
    // ...

    /* Teleop Triggers */
    // ...

    /* Auto Triggers */
    // ...

    /* State Triggers */
    // ...

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        /* Stop spamming the logs if a controller is unplugged */
        DriverStation.silenceJoystickConnectionWarning(true);

        configureButtonTriggers();

        if(kRunSysId)
            bindSysIdTriggers();

        /* Allows us to choose from all autos in the deploy directory */
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        /* Add Git Data to Elastic */
        SmartDashboard.putString("Version Info", "Branch: \"" + GitData.GIT_BRANCH + "\" Build Date: " + GitData.BUILD_DATE);
    }

    /** Use this method to define your button trigger->command mappings. */
    private void configureButtonTriggers() {
        /* Driver Controls */
        krakenSwerve.setDefaultCommand(
            new TeleopSwerve(
                krakenSwerve,
                driveController::getRightX,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX()
            )
        );

        zeroGyro.onTrue(
            krakenSwerve.zeroGyro()
            .andThen(ElasticUtil.sendInfoCommand("Zeroed gyro"))
        );

        groundPTrigger.onTrue(
            intake.moveToGround()
            .andThen(ElasticUtil.sendInfoCommand("Ground Position"))
        );

        stowPTrigger.onTrue(
            intake.returnToStowPosition()
            .andThen(ElasticUtil.sendInfoCommand("Stow Position"))
        );

        eject.onTrue(intake.ejectCoral());

        armTrough.onTrue(arm.moveToTroughPos());
        armLevel1.onTrue(arm.moveToLevel1Pos());
        armLevel2.onTrue(arm.moveToLevel2Pos());
        armLevel3.onTrue(arm.moveToLevel3Pos());

        scoreCoral.onTrue(endEffector.placeCoral());
        pickupCoral.onTrue(endEffector.pickupCoralFromSource());

        /* Operator Controls */
        elevatorTroughButton.onTrue(elevator.moveToTroughPosition());
        elevatorL1Button.onTrue(elevator.moveToL1Position());
        elevatorL2Button.onTrue(elevator.moveToL1Position());
        elevatorL3Button.onTrue(elevator.moveToL3Position());

        index.onTrue(indexer.spinRollersUntilCoralReceived());
    }
    
    private void bindSysIdTriggers() {
        Trigger sysIdReverse = driveController.leftBumper();
        Trigger sysIdDynamic = driveController.start();
        Trigger sysIdQuasistatic = driveController.back();

        sysIdDynamic.and(sysIdReverse.negate()).whileTrue(
            krakenSwerve.runSysIdDynamic(Direction.kForward)
        );

        sysIdDynamic.and(sysIdReverse).whileTrue(
            krakenSwerve.runSysIdDynamic(Direction.kReverse)
        );

        sysIdQuasistatic.and(sysIdReverse.negate()).whileTrue(
            krakenSwerve.runSysIdQuasistatic(Direction.kForward)
        );

        sysIdQuasistatic.and(sysIdReverse).whileTrue(
            krakenSwerve.runSysIdQuasistatic(Direction.kReverse)
        );
    }

    /** Use this to pass the autonomous command */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
