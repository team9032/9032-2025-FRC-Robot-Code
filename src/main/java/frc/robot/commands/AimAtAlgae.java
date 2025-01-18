package frc.robot.commands;

import frc.robot.subsystems.swerve.KrakenSwerve;

import static frc.robot.Constants.ObjectAimingConstants.*;

public class AimAtAlgae extends AimAtObject {
    public AimAtAlgae(KrakenSwerve swerve) {
        super(swerve, kAlgaeId, kAlgaeHeight);
    }
}