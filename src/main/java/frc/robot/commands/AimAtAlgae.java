package frc.robot.commands;

import frc.robot.subsystems.swerve.KrakenSwerve;

import static frc.robot.Constants.ObjectAimingConstants.*;

import java.util.function.DoubleSupplier;

public class AimAtAlgae extends AimAtObject {
    public AimAtAlgae(KrakenSwerve swerve, DoubleSupplier obstacleDistanceSup) {
        super(swerve, kAlgaeId, obstacleDistanceSup);
    }
}