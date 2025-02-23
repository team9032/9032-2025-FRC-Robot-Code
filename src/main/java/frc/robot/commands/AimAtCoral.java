package frc.robot.commands;

import frc.robot.subsystems.swerve.KrakenSwerve;

import static frc.robot.Constants.ObjectAimingConstants.*;

public class AimAtCoral extends AimAtObject {
    public AimAtCoral(KrakenSwerve swerve) {
        super(swerve, kCoralId);
    }
}