// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.automation.AutomationHandler;
import frc.robot.automation.ButtonBoardHandler;
import frc.robot.automation.Compositions;
import frc.robot.automation.ElevatorArmIntakeHandler;
import frc.robot.commands.Autos;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerve.KrakenSwerve;
import frc.robot.utils.ElasticUtil;
import frc.robot.utils.GitData;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
    private final Trigger slowMode = driveController.leftBumper().or(driveController.rightBumper());
    private final Trigger resetPerspective = driveController.b();
    private final Trigger eject = driveController.x();
    private final Trigger stowPosition = driveController.y();
    private final Trigger intakeDown = driveController.rightTrigger();
    private final Trigger intakeUp = driveController.leftTrigger();
    private final Trigger resumeAutomation = driveController.a();

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
    private final ElevatorArmIntakeHandler elevatorArmIntakeHandler = new ElevatorArmIntakeHandler(elevator, arm, intake, buttonBoard);
    private final Compositions compositions = new Compositions(elevatorArmIntakeHandler, endEffector, indexer, intake, krakenSwerve, buttonBoard);
    private final AutomationHandler automationHandler = new AutomationHandler(compositions, endEffector, buttonBoard);
    private final Command coralCyclingCommand;
    private final Command algaeCyclingCommand;

    /* Robot Mode Triggers */
    private final Trigger teleopEnabled = RobotModeTriggers.teleop();

    /* Teleop Triggers */
    private final Trigger hasCoral = new Trigger(endEffector::hasCoral);

    /* Auto Triggers */
    // ...

    /* State Triggers */
    // ...

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        /* Stop spamming the logs if a controller is unplugged */
        DriverStation.silenceJoystickConnectionWarning(true);

        /* Setup automation */
        coralCyclingCommand = automationHandler.coralResumeCommand()//automationHandler.coralResumeCommand()
            .until(this::driverWantsOverride)
            .andThen(new ScheduleCommand(elevatorArmIntakeHandler.moveToStowPositions())
                .onlyIf(endEffector::hasCoral)
            )
            .onlyIf(buttonBoard::hasQueues);

        algaeCyclingCommand = automationHandler.algaeResumeCommand()
            .until(this::driverWantsOverride);

        buttonBoard.getEnableCoralModeTrigger()
            .toggleOnTrue(coralCyclingCommand);

        buttonBoard.getEnableAlgaeModeTrigger().onTrue(//TODO auto algae
            Commands.sequence(
                elevatorArmIntakeHandler.prepareForAlgaeIntaking(),
                endEffector.pickupAlgae()  
            )
        );

        // buttonBoard.getEnableAlgaeModeTrigger()
        //     .toggleOnTrue(algaeCyclingCommand.onlyIf(buttonBoard::hasQueues));

        // buttonBoard.getAutoIntakeTrigger().onTrue(
        //     compositions.driveToSource(false, false)
        //     .onlyIf(() -> !endEffector.hasCoral())
        //     .until(this::driverWantsOverride)
        // );     

        if(kRunSysId)
            bindSysIdTriggers();
        
        else    
            configureButtonTriggers();

        configureDefaultCommands();

        bindRobotModeTriggers();

        bindTeleopTriggers();     

        /* Allows us to choose from all autos in the deploy directory */
        autoChooser = new SendableChooser<>();
        autoChooser.addOption("3 Coral Left", Autos.threeCoralLeft(elevator, arm, endEffector, krakenSwerve, intake, indexer));
        autoChooser.addOption("1 Coral Right", Autos.threeCoralRight(elevator, arm, endEffector, krakenSwerve, intake, indexer));
        autoChooser.setDefaultOption("Do Nothing", Commands.none());

        SmartDashboard.putData("Auto Chooser", autoChooser);

        /* Add Git Data to Elastic */
        SmartDashboard.putString("Version Info", "Branch: \"" + GitData.GIT_BRANCH + "\" Build Date: " + GitData.BUILD_DATE);
    }

    private boolean isSlowModeEnabled() {
        if (slowMode.getAsBoolean()) 
            return true;
        else 
            return elevator.getElevatorOverSlowModeHeight();
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
                () -> -driveController.getLeftX(),
                () -> isSlowModeEnabled()
            )
        );  
    }

    /** Use this method to define your button trigger->command mappings. */
    private void configureButtonTriggers() {
        /* Driver Controls */      
        resetPerspective.onTrue(
            krakenSwerve.resetPerspective()
            .andThen(ElasticUtil.sendInfoCommand("Reset perspective"))
        );

        eject.onTrue(
            intake.ejectCoral()
        );

        stowPosition.onTrue(
            elevatorArmIntakeHandler.moveToStowPositions()
        );

        intakeDown.onTrue(
            compositions.intakeCoralToEndEffector()
        );

        intakeUp.onTrue(
            compositions.cancelIntake()
        );

        resumeAutomation.onTrue(
            coralCyclingCommand
        );

        /* Manual Controls:
         * 
         * Manual 1 - eject coral from intake
         * Manual 2 - eject coral from indexer
         * Manual 3 - manual put arm and elevator to reef positions
         * Manual 6 - score coral
         * Manual 7 - return to stow positions
         * Manual 8 - intake up
         * Manual 9 - algae eject
         * Manual 10 - algae low
         * Manual 11 - algae high
         * Manual 12 - intake down
         * Manual 13 - source intake
         * 
        */
        buttonBoard.manual1.onTrue(
            intake.ejectCoral()
        );

        buttonBoard.manual2.onTrue(
            Commands.sequence(
                intake.returnToStowPosition(),
                Commands.waitSeconds(0.5),
                indexer.eject()
            )
        );

        buttonBoard.manual3.onTrue(
            elevatorArmIntakeHandler.prepareForCoralScoring()
        );

        buttonBoard.manual6.onTrue(
            Commands.sequence(
                endEffector.placeCoral(),
                elevatorArmIntakeHandler.moveToIntakePosition(false)
            )
        );

        buttonBoard.manual7.onTrue(
            elevatorArmIntakeHandler.moveToStowPositions()
        );

        buttonBoard.manual8.onTrue(
            compositions.cancelIntake()
        );

        buttonBoard.manual9.onTrue(
            Commands.sequence(
                disableAutomation(),
                endEffector.outtakeProcessorAlgae()
            )
        );

        buttonBoard.manual10.onTrue(
            Commands.sequence(
                elevatorArmIntakeHandler.prepareForAlgaeScoring(),
                endEffector.pickupAlgae()  
            )
        );

        // buttonBoard.manual11.onTrue(
        //     Commands.sequence(
        //         disableAutomation(),
        //         elevator.moveToHighAlgaePosition(),
        //         Commands.waitUntil(elevator::atSetpoint),
        //         arm.moveToHighAlgaePos()
        //     )
        // );

        buttonBoard.manual12.onTrue(
            compositions.intakeCoralToEndEffector()
        );

        buttonBoard.manual13.onTrue(
            Commands.sequence(
                arm.moveToSourcePos(),
                elevator.moveToSourcePosition(),
                endEffector.pickupCoralFromSource()
            )
        );
    }

    private Command disableAutomation() {
        return Commands.runOnce(() -> coralCyclingCommand.cancel());
    }

    /** Runs every loop cycle */
    public void robotPeriodic() {
        buttonBoard.update(coralCyclingCommand.isScheduled(), algaeCyclingCommand.isScheduled());
    }

    /** Bind robot mode triggers here */
    private void bindRobotModeTriggers() {
        teleopEnabled.onTrue(
            compositions.resetStates()
            .andThen(elevatorArmIntakeHandler.holdPositions())
        );
    }

    private void bindTeleopTriggers() {
        hasCoral.onTrue(rumble());
    }

    private Command rumble() {
        return Commands.sequence(
            Commands.runOnce(() -> driveController.setRumble(RumbleType.kBothRumble, 1.0)),
            Commands.waitSeconds(kRumbleTime),
            Commands.runOnce(() -> driveController.setRumble(RumbleType.kBothRumble, 0.0))
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
