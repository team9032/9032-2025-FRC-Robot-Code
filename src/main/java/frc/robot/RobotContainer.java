// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.automation.AutomationHandler;
import frc.robot.automation.ButtonBoardHandler;
import frc.robot.automation.Compositions;
import frc.robot.automation.ElevatorArmIntakeHandler;
import frc.robot.automation.GroundCoralTracking;
import frc.robot.automation.PathfindingHandler;
import frc.robot.commands.Autos;
import frc.robot.commands.DriverAssistedAutoIntake;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LED.State;
import frc.robot.subsystems.swerve.KrakenSwerve;
import frc.robot.utils.ElasticUtil;
import frc.robot.utils.GitData;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
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

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;

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
    //private final Trigger slowMode = driveController.leftBumper().or(driveController.rightBumper());
    private final Trigger resetPerspective = driveController.b();
    private final Trigger eject = driveController.x();
    private final Trigger stowPosition = driveController.y();
    private final Trigger intakeDown = driveController.rightTrigger();
    private final Trigger intakeUp = driveController.leftTrigger();
    private final Trigger resumeAutomation = driveController.a();
    private final Trigger pulseIntake = driveController.povRight();
    private final Trigger alignReefLeft = driveController.leftBumper();
    private final Trigger alignReefRight = driveController.rightBumper();

    /* Operator Controller Buttons */


    /* Subsystems */
    private final LED led = new LED();
    private final Intake intake = new Intake();
    private final Arm arm = new Arm();
    private final KrakenSwerve krakenSwerve = new KrakenSwerve();
    private final Elevator elevator = new Elevator();
    private final Transfer transfer = new Transfer();
    private final Climber climber = new Climber();
    private final EndEffector endEffector = new EndEffector();

    /* Dashboard */
    private final SendableChooser<Command> autoChooser;

    /* Automation */
    private final ButtonBoardHandler buttonBoard = new ButtonBoardHandler();
    private final ElevatorArmIntakeHandler elevatorArmIntakeHandler = new ElevatorArmIntakeHandler(elevator, arm, intake);
    private final Compositions compositions = new Compositions(elevatorArmIntakeHandler, endEffector, transfer, intake, climber, krakenSwerve, buttonBoard);
    private final AutomationHandler automationHandler = new AutomationHandler(compositions, endEffector, buttonBoard);
    private final Command coralCyclingCommand;
    private final Command algaeCyclingCommand;
    private final GroundCoralTracking groundCoralTracking = new GroundCoralTracking(krakenSwerve.getLocalization(), buttonBoard);

    /* Robot Mode Triggers */
    private final Trigger teleopEnabled = RobotModeTriggers.teleop();
    private final Trigger disabled = RobotModeTriggers.disabled();
    private final Trigger enabled = RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop());

    /* Teleop Triggers */
    private final Trigger hasCoral = new Trigger(endEffector::hasCoral);
    private final Trigger coralCyclingCommandScheduled;
    private final Trigger algaeCyclingCommandScheduled;
    private final Trigger groundCoralOnFarReef = new Trigger(groundCoralTracking::coralBlockingAlignmentOnFarReef);

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        /* Stop spamming the logs if a controller is unplugged */
        DriverStation.silenceJoystickConnectionWarning(true);

        /* Warm up PathPlanner */
        PathfindingCommand.warmupCommand().schedule();
        FollowPathCommand.warmupCommand().schedule();

        /* Setup automation */
        coralCyclingCommand = automationHandler.coralResumeCommand()
            .until(this::driverWantsOverride)
            .andThen(
                new ScheduleCommand(elevatorArmIntakeHandler.moveToStowPositions())
                    .onlyIf(endEffector::hasCoral)
        );

        algaeCyclingCommand = automationHandler.algaeResumeCommand()
            .until(this::driverWantsOverride)
            .andThen(
                compositions.stopRollers().asProxy()
                    .onlyIf(() -> !endEffector.hasAlgae())
            );

        buttonBoard.getEnableCoralModeTrigger()
            .toggleOnTrue(coralCyclingCommand);

        buttonBoard.getEnableAlgaeModeTrigger()
            .onTrue(algaeCyclingCommand);

        buttonBoard.getAutoIntakeTrigger().onTrue(
            compositions.intakeNearestCoral(true)
            .until(this::driverWantsOverride)
        );     

        /* Bind Triggers */
        coralCyclingCommandScheduled = new Trigger(coralCyclingCommand::isScheduled);
        algaeCyclingCommandScheduled = new Trigger(algaeCyclingCommand::isScheduled);

        if(kRunSysId)
            bindSysIdTriggers();
        
        else    
            configureButtonTriggers();

        configureDefaultCommands();

        bindRobotModeTriggers();

        bindTeleopTriggers();   

        /* Allows us to choose from all autos in the deploy directory */
        autoChooser = new SendableChooser<>();
        autoChooser.addOption("3 Coral Left", Autos.fourCoralLeft(elevatorArmIntakeHandler, endEffector, krakenSwerve, compositions, false));
        autoChooser.addOption("3 Coral Right", Autos.fourCoralLeft(elevatorArmIntakeHandler, endEffector, krakenSwerve, compositions, true));
        autoChooser.addOption("1 Coral, 2 Algae Center", Autos.oneCoralTwoAlgaeCenter(elevatorArmIntakeHandler, endEffector, krakenSwerve, compositions));
        autoChooser.addOption("3 Coral Left Dynamic", Autos.dynamicCoralAuto(compositions, elevatorArmIntakeHandler));
        autoChooser.setDefaultOption("Do Nothing", Commands.none());

        SmartDashboard.putData("Auto Chooser", autoChooser);

        /* Add Git Data to Elastic */
        SmartDashboard.putString("Version Info", "Branch: \"" + GitData.GIT_BRANCH + "\" Build Date: " + GitData.BUILD_DATE);

        /* Add coast button */
        SmartDashboard.putData(compositions.coastAll().withName("Coast All"));

        /* Switch LEDs to disabled or low battery */
        if (RobotController.getBatteryVoltage() < kLowStartingBatteryVoltage)
            led.setState(State.LOW_BATTERY); 

        else
            led.setState(State.DISABLED); 
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
                () -> false//TODO remove slowMode?
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
            compositions.cancelClimbAndStow()
        );

        intakeDown.onTrue(
            Commands.either(
                elevatorArmIntakeHandler.moveIntakeDown(), 
                compositions.intakeCoralToEndEffector(true),
                endEffector::hasCoral
            )
        );

        intakeDown.whileTrue(
            new DriverAssistedAutoIntake(
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                krakenSwerve
            )
            .onlyIf(() -> !endEffector.hasCoral())
        );

        intakeUp.onTrue(
            Commands.either(
                elevatorArmIntakeHandler.moveIntakeUp(), 
                compositions.cancelIntake(),
                () -> endEffector.hasCoral() || endEffector.hasAlgae()
            )
        );

        resumeAutomation.and(endEffector::hasCoral).onTrue(//Prevent shooting coral out of the end effector
            coralCyclingCommand
        );

        pulseIntake.onTrue(compositions.pulseIntake());

        alignReefRight.onTrue(PathfindingHandler.pathToClosestReefBranch(krakenSwerve, false));

        alignReefLeft.onTrue(PathfindingHandler.pathToClosestReefBranch(krakenSwerve, true));

        /* Manual Controls:
         * 
         * Manual 1 - eject coral from intake
         * Manual 2 - eject coral from indexer
         * Manual 3 - manual put arm and elevator to reef positions
         * Manual 6 - score coral
         * Manual 7 - return to stow positions
         * Manual 8 - intake up
         * Manual 9 - algae eject
         * Manual 10 - algae score pos
         * Manual 11 - algae ground intake
         * Manual 12 - intake down
         * 
        */
        buttonBoard.manual1.onTrue(
            intake.ejectCoral()
        );

        buttonBoard.manual2.onTrue(
            Commands.sequence(
                intake.returnToStowPosition(),
                Commands.waitSeconds(0.5),
                transfer.eject()
            )
        );

        buttonBoard.manual3.onTrue(
            elevatorArmIntakeHandler.prepareForCoralScoring(buttonBoard::getSelectedReefLevel)
        );

        buttonBoard.manual6.onTrue(
            Commands.sequence(
                endEffector.scoreCoral(buttonBoard::getSelectedReefLevel).asProxy(),
                elevatorArmIntakeHandler.moveToIntakePosition()
            )
        );

        buttonBoard.manual7.onTrue(
            elevatorArmIntakeHandler.moveToStowPositions()
            .andThen(compositions.stopRollers()
                .onlyIf(() -> !endEffector.hasAlgae())
            )
        );

        buttonBoard.manual8.onTrue(
            compositions.cancelIntake()
        );

        buttonBoard.manual9.onTrue(
            Commands.sequence(
                endEffector.outtakeProcessorAlgae(),
                elevatorArmIntakeHandler.moveToStowPositions()
            )
        );

        buttonBoard.manual10.onTrue(
            Commands.sequence(
                elevatorArmIntakeHandler.prepareForNetAlgaeScoring(),
                endEffector.intakeAlgae()  
            )
        );

        buttonBoard.manual11.onTrue(
            Commands.sequence(
                elevatorArmIntakeHandler.prepareForAlgaeGroundIntaking(),
                endEffector.intakeAlgae()
            )
        );

        buttonBoard.manual12.onTrue(
            compositions.intakeCoralToEndEffector(true)
        );
    }

    /** Runs every loop cycle */
    public void robotPeriodic() {
        buttonBoard.update(coralCyclingCommand.isScheduled(), algaeCyclingCommand.isScheduled());
        
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

        /* Display CAN errors on the LEDs */
        var currentCANStatus = RobotController.getCANStatus();
        if (currentCANStatus.receiveErrorCount > 0 || currentCANStatus.transmitErrorCount > 0)
            led.displayError();
    }

    /** Bind robot mode triggers here */
    private void bindRobotModeTriggers() {
        teleopEnabled.onTrue(
            compositions.stopRollers()
            .andThen(elevatorArmIntakeHandler.holdPositions())
        );

        enabled.onTrue(
            led.setStateCommand(State.ENABLED)
        );

        disabled.onTrue(
            led.setStateCommand(State.DISABLED)
        );
    }

    private void bindTeleopTriggers() {
        hasCoral.onTrue(rumble());

        coralCyclingCommandScheduled.onTrue(Commands.runOnce(() -> buttonBoard.setCoralAimingLEDs(led)));
        coralCyclingCommandScheduled.onFalse(
            Commands.either(
                led.setStateCommand(State.ENABLED), 
                led.setStateCommand(State.DISABLED),
                enabled
            )
        );

        algaeCyclingCommandScheduled.onTrue(led.setStateCommand(State.ALGAE));
        algaeCyclingCommandScheduled.onFalse(
            Commands.either(
                led.setStateCommand(State.ENABLED), 
                led.setStateCommand(State.DISABLED),
                enabled
            )
        );

        groundCoralOnFarReef.and(coralCyclingCommandScheduled).onTrue(
            rumble()
            .alongWith(
                led.setStateCommand(State.CORAL_BLOCKING_ALIGNMENT)
            )
        );
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
