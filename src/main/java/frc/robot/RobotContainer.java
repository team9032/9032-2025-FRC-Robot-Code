// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.swerve.KrakenSwerve;
import frc.robot.utils.ElasticUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController driveController = new CommandXboxController(DriverConstants.kDriveControllerPort);

    /* Drive Controller Buttons */
    private final Trigger zeroGyro = driveController.b();

    /* Operator Controller Buttons */
    //...

    /* Subsystems */
    private final KrakenSwerve krakenSwerve = new KrakenSwerve();

    /* Dashboard */


    /* Robot Mode Triggers */
    //...

    /* Teleop Triggers */
    //...

    /* Auto Triggers */
    //...

    /* State Triggers */
    //...

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
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
    
    /** Use this to pass the autonomous command */
    public Command getAutonomousCommand() {
        return null;
    }
}
