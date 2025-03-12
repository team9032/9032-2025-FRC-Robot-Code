// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.automation.AutomationHandler;
import frc.robot.automation.ButtonBoardHandler;
import frc.robot.automation.Compositions;
import frc.robot.commands.AimAtCoral;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerve.KrakenSwerve;
import frc.robot.utils.ElasticUtil;
import frc.robot.utils.GitData;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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

    /* Drive Controller Buttons */
    private final Trigger scoreCoral = driveController.a();
    private final Trigger pickupCoral = driveController.y();
    private final Trigger intakeAlgae = driveController.x();
    private final Trigger intakeCoral = driveController.povLeft();

    private final Trigger algaeL1 = driveController.leftBumper();
    private final Trigger algaeL2 = driveController.rightBumper();
    private final Trigger resetPerspective = driveController.b();

    /* Operator Controller Buttons */


    /* Subsystems */
    private final Intake intake = new Intake();
    private final Arm arm = new Arm();
    private final KrakenSwerve krakenSwerve = new KrakenSwerve();
    private final Elevator elevator = new Elevator();
    private final Indexer indexer = new Indexer();
    // private final Climber climber = new Climber();
    private final EndEffector endEffector = new EndEffector();

    /* Dashboard */
    private final SendableChooser<Command> autoChooser;

    /* Automation */
    private final ButtonBoardHandler buttonBoard = new ButtonBoardHandler();
    private final Compositions compositions = new Compositions(arm, elevator, endEffector, indexer, intake, krakenSwerve, buttonBoard);
    private final AutomationHandler automationHandler = new AutomationHandler(compositions, arm, elevator, endEffector, indexer, intake, krakenSwerve, buttonBoard);
    private final Command automationCommand;

    /* Robot Mode Triggers */
    private final Trigger teleopEnabled = RobotModeTriggers.teleop();

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

        if(kRunSysId)
            bindSysIdTriggers();
        
        else    
            configureButtonTriggers();

        configureDefaultCommands();

        bindRobotModeTriggers();

        /* Setup automation */
        automationCommand = automationHandler.automationResumeCommand()
            .until(this::driverWantsOverride);

        buttonBoard.getEnableAutomaticModeTrigger()
            .toggleOnTrue(automationCommand.onlyIf(buttonBoard::hasQueues));

        buttonBoard.getAutoIntakeTrigger().onTrue(
            compositions.getCoralSequence(false, false)
            .until(this::driverWantsOverride)
        );  

        /* Allows us to choose from all autos in the deploy directory */
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        /* Add Git Data to Elastic */
        SmartDashboard.putString("Version Info", "Branch: \"" + GitData.GIT_BRANCH + "\" Build Date: " + GitData.BUILD_DATE);
    }

    private boolean driverWantsOverride() {
        return Math.abs(driveController.getRightX()) > kOverrideAutomationThreshold || 
            Math.abs(driveController.getLeftX()) > kOverrideAutomationThreshold ||    
            Math.abs(driveController.getLeftY()) > kOverrideAutomationThreshold;
    }

    private void configureDefaultCommands() {
        krakenSwerve.setDefaultCommand(
            new TeleopSwerve(
                krakenSwerve,
                driveController::getRightX,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX()
            )
        );  
    }

    /** Use this method to define your button trigger->command mappings. */
    private void configureButtonTriggers() {
        /* Driver Controls */      
        intakeAlgae.onTrue(endEffector.pickupAlgae());

        scoreCoral.onTrue(endEffector.placeCoral());

        pickupCoral.onTrue(
            Commands.sequence(
                elevator.moveToSourcePosition(),
                arm.moveToSourcePos(),
                endEffector.pickupCoralFromSource(),
                arm.moveToStowPos(),
                elevator.moveToIndexerPosition(),
                new ScheduleCommand(endEffector.holdCoral())
            )
        );

        algaeL1.onTrue(
            arm.moveToLowAlgaePos()
            .andThen(elevator.moveToLowAlgaePosition())
        );

        resetPerspective.onTrue(
            krakenSwerve.resetPerspective()
            .andThen(ElasticUtil.sendInfoCommand("Reset perspective"))
        );

        /* Operator Controls */
    }

    /** Runs every loop cycle */
    public void robotPeriodic() {
        buttonBoard.update(automationCommand.isScheduled());
    }

    /** Bind robot mode triggers here */
    private void bindRobotModeTriggers() {
        teleopEnabled.onTrue(
            Commands.sequence(
                intake.stopIntaking(),
                indexer.stopRollers(),
                endEffector.stopRollers()
            )
        );
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
