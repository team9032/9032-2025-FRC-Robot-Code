package frc.robot.utils;

import static frc.robot.Constants.PathFollowingConstants.kRobotRelativeClosedLoopDriveRequest;

import java.util.List;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.KrakenSwerve;

public class WheelRadiusFinder {//TODO this is broken
    private final KrakenSwerve swerve;

    private double initialYaw;
    private List<Double> initialWheelPositions;

    public WheelRadiusFinder(KrakenSwerve swerve) {
        this.swerve = swerve;
    }

    private Command stop() {
        return Commands.runOnce(() -> swerve.setControl(kRobotRelativeClosedLoopDriveRequest.withSpeeds(new ChassisSpeeds())), swerve);
    }

    private Command rotate() {
        return Commands.runOnce(
            () -> swerve.setControl(
                kRobotRelativeClosedLoopDriveRequest.withSpeeds(new ChassisSpeeds(0.0, 0.0, Math.PI / 4))
            ), 
            swerve
        );
    }

    private void recordInitialValues() {
        initialWheelPositions = swerve.getWheelPositionsRadians();
        initialYaw = swerve.getGyroYaw();        
    }

    private void calculateWheelRadius() {
        var currentWheelPositions = swerve.getWheelPositionsRadians();

        double rotationalDistance = Math.abs(swerve.getGyroYaw() - initialYaw) * swerve.getDrivebaseRadius();

        double[] wheelRadii = new double[4];
        for (int i = 0; i < 4; i++) {
            double wheelPositionDelta = Math.abs(currentWheelPositions.get(i) - initialWheelPositions.get(i));

            wheelRadii[i] = rotationalDistance / wheelPositionDelta;

            System.out.println("Module " + i + " has wheel radius " + wheelRadii[i]);
        }

        double sum = 0.0;
        for (double radius : wheelRadii)
            sum += radius;

        double averageRadius = sum / 4.0;

        System.out.println("Average radius is " + averageRadius + " (m) " + Units.metersToInches(averageRadius) + " (in)");
    }

    public Command findRadius() {
        return Commands.sequence(
            /* Rotate quickly and come to a stop to make sure the swerve modules are pointing in the right direction */
            rotate(),
            Commands.waitSeconds(0.5),
            stop(),
            Commands.waitSeconds(1.0),
            /* Record inital values */
            Commands.runOnce(() -> recordInitialValues()),
            /* Rotate and stop to gather data */
            rotate(),
            Commands.waitSeconds(10.0),
            stop(),
            Commands.waitSeconds(2.0),
            /* Calculate wheel radius */
            Commands.runOnce(() -> calculateWheelRadius())
        );
    }
}
