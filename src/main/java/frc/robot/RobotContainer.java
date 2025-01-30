// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.swerve.KrakenSwerve;
import frc.robot.utils.ElasticUtil;
import com.pathplanner.lib.auto.AutoBuilder;
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

    /* Drive Controller Buttons */
    private final Trigger zeroGyro = driveController.b();

    /* Operator Controller Buttons */
    //...

    /* Subsystems */
    private final KrakenSwerve krakenSwerve = new KrakenSwerve();

    /* Dashboard */
    private final SendableChooser<Command> autoChooser;

    /* Robot Mode Triggers */
    //...

    /* Teleop Triggers */
    //...

    /* Auto Triggers */
    //...

    /* State Triggers */
    //...

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        configureButtonTriggers();

        if(kRunSysId)
            bindSysIdTriggers();

        /* Allows us to choose from all autos in the deploy directory */
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
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
            .andThen(ElasticUtil.sendInfoCommand("Zeroed Gyro"))
        );

        /* Operator Controls */
        //...
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
        return null;
    }
}
