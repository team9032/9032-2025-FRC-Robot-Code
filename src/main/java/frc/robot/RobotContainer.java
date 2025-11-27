// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.automation.ButtonBoardHandler;
import frc.robot.automation.Compositions;
import frc.robot.automation.ElevatorArmIntakeHandler;
import frc.robot.automation.PathfindingHandler;
import frc.robot.automation.ButtonBoardHandler.ReefLevel;
import frc.robot.commands.Autos;
import frc.robot.commands.RotationalDriveToCoral;
import frc.robot.commands.RotationalIntakeDriverAssist;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.State;
import frc.robot.subsystems.swerve.KrakenSwerve;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.utils.CANivoreReader;
import frc.robot.utils.ElasticUtil;
import frc.robot.utils.FieldUtil;
import frc.robot.utils.GitData;
import frc.robot.utils.WheelRadiusFinder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static frc.robot.Constants.DriverConstants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
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
    private final Trigger resetPerspective = driveController.povDown();
    private final Trigger deployClimber = driveController.povUp();
    private final Trigger ejectIntake = driveController.x();
    private final Trigger stowAndCancelClimb = driveController.y();
    private final Trigger intakeDown = driveController.rightTrigger();
    private final Trigger intakeUp = driveController.leftTrigger();
    private final Trigger algaeGroundIntake = driveController.a();
    private final Trigger algaeReefIntakeOrNetScore = driveController.b();
    private final Trigger alignAndScoreCoralLeft = driveController.leftBumper();
    private final Trigger alignAndScoreCoralRight = driveController.rightBumper();

    private final Trigger reefLevelUp = driveController.rightStick().and(alignAndScoreCoralLeft.or(alignAndScoreCoralRight).negate());
    private final Trigger reefLevelDown = driveController.leftStick().and(alignAndScoreCoralLeft.or(alignAndScoreCoralRight).negate());

    /* Subsystems */
    private final LEDs leds = new LEDs();
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

    /* Robot Mode Triggers */
    private final Trigger teleopEnabled = RobotModeTriggers.teleop();
    private final Trigger disabled = RobotModeTriggers.disabled();
    private final Trigger enabled = RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop());

    /* Teleop Triggers */
    private final Trigger hasCoral = new Trigger(endEffector::hasCoral).debounce(kHasCoralDebounceTime);
    private Trigger coralCyclingCommandScheduled;
    private Trigger algaeCyclingCommandScheduled;

    private final CANBus canBus = new CANBus(kCANBusName);
    private final CANivoreReader canivoreReader = new CANivoreReader(canBus);

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        /* Stop spamming the logs if a controller is unplugged */
        DriverStation.silenceJoystickConnectionWarning(true);

        /* Warm up PathPlanner */
        PathfindingCommand.warmupCommand().schedule();
        FollowPathCommand.warmupCommand().schedule(); 

        /* Bind Triggers */
        if(kRunSysId)
            bindSysIdTriggers();
        
        else {
            configureButtonTriggers();

            bindTeleopTriggers();   
        }
        
        configureDefaultCommands();

        bindRobotModeTriggers();

        /* Allows us to choose from all autos in the deploy directory */
        autoChooser = new SendableChooser<>();
        autoChooser.addOption("4 Coral Left", Autos.left4CoralAuto(compositions));
        autoChooser.addOption("4 Coral Right", Autos.right4CoralAuto(compositions));
        autoChooser.addOption("1 Coral, 1 algae center", Autos.center1Coral1AlgaeAuto(compositions, krakenSwerve));
        autoChooser.setDefaultOption("Do Nothing", Commands.none());

        SmartDashboard.putData("Auto Chooser", autoChooser);

        /* Add Git Data to Elastic */
        SmartDashboard.putString("Version Info", "Branch: \"" + GitData.GIT_BRANCH + "\" Build Date: " + GitData.BUILD_DATE);

        /* Add coast button */
        SmartDashboard.putData(compositions.coastAll().withName("Coast All"));

        /* Switch LEDs to disabled or low battery */
        if (RobotController.getBatteryVoltage() < kLowStartingBatteryVoltage)
            leds.setState(State.LOW_BATTERY); 

        else
            leds.setState(State.DISABLED); 
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
        driveController.povLeft().whileTrue(PathfindingHandler.testPathDefered(krakenSwerve));//TODO remove

        resetPerspective.onTrue(
            krakenSwerve.resetPerspective()
            .andThen(ElasticUtil.sendInfoCommand("Reset perspective"))
        );

        ejectIntake.onTrue(
            compositions.ejectIntake()
        );

        stowAndCancelClimb.onTrue(
            compositions.cancelClimbAndStow()
        );

        intakeDown.onTrue(
            Commands.either(
                elevatorArmIntakeHandler.moveIntakeDown(), 
                Commands.either(
                    compositions.intakeCoralToCradle(), 
                    compositions.intakeCoralToEndEffector(), 
                    endEffector::hasAlgae
                ),
                endEffector::hasCoral
            )
        );

        Command intakeCommand = kUseAutoIntake ? 
            new RotationalDriveToCoral(krakenSwerve) : 
            new RotationalIntakeDriverAssist(
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                krakenSwerve
        );

        intakeDown.debounce(kIntakeDriverAssistStartTime).whileTrue(
            intakeCommand
            .onlyIf(() -> !endEffector.hasCoral())
        );

        intakeUp.onTrue(
            Commands.either(
                elevatorArmIntakeHandler.moveIntakeUp(), 
                compositions.cancelIntake(),
                () -> endEffector.hasCoral() || endEffector.hasAlgae()
            )
        );

        algaeGroundIntake.onTrue(compositions.intakeGroundAlgae());

        deployClimber.onTrue(leds.setStateCommand(State.CLIMBING).andThen(compositions.climb()));

        reefLevelDown.onTrue(Commands.runOnce(() -> buttonBoard.decrementReefLevel()).ignoringDisable(true));
        reefLevelUp.onTrue(Commands.runOnce(() -> buttonBoard.incrementReefLevel()).ignoringDisable(true));
        reefLevelDown.or(reefLevelUp).onTrue(setLEDEnabledState());

        /* Coral cycling commands */
        Command alignAndScoreCoralLeftCommand = 
            Commands.either(
                compositions.scoreL1(false),
                compositions.alignToReefAndScore(true, buttonBoard::getSelectedReefLevel, () -> !alignAndScoreCoralLeft.getAsBoolean(), rumble()), 
                () -> buttonBoard.getSelectedReefLevel().equals(ReefLevel.L1)
            );
        alignAndScoreCoralLeft.onTrue(alignAndScoreCoralLeftCommand);

        Command alignAndScoreCoralRightCommand = 
            Commands.either(
                compositions.scoreL1(true),
                compositions.alignToReefAndScore(false, buttonBoard::getSelectedReefLevel, () -> !alignAndScoreCoralRight.getAsBoolean(), rumble()), 
                () -> buttonBoard.getSelectedReefLevel().equals(ReefLevel.L1)
            );
        alignAndScoreCoralRight.onTrue(alignAndScoreCoralRightCommand);

        coralCyclingCommandScheduled = new Trigger(() -> alignAndScoreCoralRightCommand.isScheduled() || alignAndScoreCoralLeftCommand.isScheduled());

        /* Algae cycling commands */
        Command algaeReefIntakeOrNetScoreCommand = Commands.either(
            compositions.scoreAlgaeInNet(() -> !algaeReefIntakeOrNetScore.getAsBoolean()), 
            compositions.intakeNearestAlgaeFromReef(() -> !algaeReefIntakeOrNetScore.getAsBoolean()), 
            endEffector::hasAlgae
        );
        algaeReefIntakeOrNetScore.onTrue(algaeReefIntakeOrNetScoreCommand);

        algaeCyclingCommandScheduled = new Trigger(() -> algaeReefIntakeOrNetScoreCommand.isScheduled());

        /* Manual Controls:
         * 
         * Auto Intake (hold) - drive to coral while intaking
         * Manual 1 - eject coral from intake
         * Manual 2 - eject coral from indexer
         * Manual 3 - manual put arm and elevator to reef positions
         * Manual 4 - reintake end effector
         * Manual 5 - climb
         * Manual 6 - score coral
         * Manual 7 - return to stow positions
         * Manual 8 - intake up
         * Manual 9 - algae eject
         * Manual 10 - algae score pos
         * Manual 11 - algae ground intake
         * Manual 12 - intake down
         * 
        */
        buttonBoard.getAutoIntakeTrigger().whileTrue(
            new RotationalDriveToCoral(krakenSwerve)
                .alongWith(compositions.intakeCoralToEndEffector())
        );    

        buttonBoard.manual1.onTrue(compositions.ejectIntake());

        buttonBoard.manual2.onTrue(
            Commands.sequence(
                intake.returnToStowPosition(),
                Commands.waitSeconds(0.5),
                transfer.eject()
            )
        );

        buttonBoard.manual3.onTrue(elevatorArmIntakeHandler.prepareForBranchCoralScoring(buttonBoard::getSelectedReefLevel));

        buttonBoard.manual4.onTrue(endEffector.startRollersForPickup());

        buttonBoard.manual5.onTrue(leds.setStateCommand(State.CLIMBING).andThen(compositions.climb()));

        buttonBoard.manual6.onTrue(compositions.placeCoralOnBranch(buttonBoard::getSelectedReefLevel));

        buttonBoard.manual7.onTrue(compositions.cancelClimbAndStow());

        buttonBoard.manual8.onTrue(compositions.cancelIntake());

        buttonBoard.manual9.onTrue(endEffector.outtakeProcessorAlgae());

        buttonBoard.manual10.onTrue(elevatorArmIntakeHandler.prepareForNetAlgaeScoring());

        buttonBoard.manual11.onTrue(compositions.intakeGroundAlgae());

        buttonBoard.manual12.onTrue(compositions.intakeCoralToEndEffector());
    }

    /** Runs every loop cycle */
    public void robotPeriodic() {
        buttonBoard.update();
        
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());

        SmartDashboard.putNumber("Robot To Reef Distance", FieldUtil.getRobotToReefDistance(krakenSwerve.getLocalization()));

        /* Display CAN errors on the LEDs */
        var currentCANStatus = canivoreReader.getStatus();
        if (!currentCANStatus.Status.equals(StatusCode.OK) || currentCANStatus.TEC > 0 || currentCANStatus.REC > 0)
            leds.displayError();

        SmartDashboard.putNumber("CAN Usage", currentCANStatus.BusUtilization);
    }

    /** Bind robot mode triggers here */
    private void bindRobotModeTriggers() {
        teleopEnabled.onTrue(
            compositions.stopRollers()
            .andThen(
                elevatorArmIntakeHandler.holdPositions(),
                compositions.initClimberIfNeeded()
            )
        );

        enabled.onTrue(
            leds.setEnabledStateFromReefLevel(buttonBoard::getSelectedReefLevel)
        );

        disabled.onTrue(
            leds.setStateCommand(State.DISABLED)
        );
    }

    private void bindTeleopTriggers() {
        hasCoral.onTrue(rumble());

        coralCyclingCommandScheduled.onTrue(leds.setScoringStateFromReefLevel(buttonBoard::getSelectedReefLevel));
        coralCyclingCommandScheduled.onFalse(setLEDEnabledState());
        algaeCyclingCommandScheduled.onTrue(leds.setStateCommand(State.ALGAE));
        algaeCyclingCommandScheduled.onFalse(setLEDEnabledState());
    }

    private Command setLEDEnabledState() {
        return Commands.either(
            leds.setEnabledStateFromReefLevel(buttonBoard::getSelectedReefLevel), 
            leds.setStateCommand(State.DISABLED),
            enabled
        );
    }

    private Command rumble() {
        return Commands.sequence(
            Commands.runOnce(() -> driveController.setRumble(RumbleType.kBothRumble, 1.0)),
            Commands.waitSeconds(kRumbleTime)
        )
        .finallyDo(() -> driveController.setRumble(RumbleType.kBothRumble, 0.0));
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

        driveController.a().onTrue(new WheelRadiusFinder(krakenSwerve).findRadius());
    }

    /** Use this to pass the autonomous command */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
