package frc.robot.commands;

import static frc.robot.Constants.PathFollowingConstants.kRobotRelativeClosedLoopDriveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.KrakenSwerve;

public class FindWheelRadius extends Command {
    private final KrakenSwerve swerve;

    private final Timer timer = new Timer();

    private double initialYaw;
    private double[] initialWheelPositions = new double[4];

    public FindWheelRadius(KrakenSwerve swerve) {
        this.swerve = swerve;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        for (int i = 0; i < 4; i++) 
            initialWheelPositions[i] = swerve.getWheelPositionRadians(i);

        initialYaw = swerve.getGyroYaw();

        swerve.setControl(
            kRobotRelativeClosedLoopDriveRequest.withSpeeds(new ChassisSpeeds(0.0, 0.0, Math.PI / 4))
        );

        timer.start();
    }

    @Override
    public void execute() {
        /* Stop early to allow time to slow down */
        if (timer.hasElapsed(10.0)) {
            swerve.setControl(
                kRobotRelativeClosedLoopDriveRequest.withSpeeds(new ChassisSpeeds())
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();

        double rotationalDistance = Math.abs(swerve.getGyroYaw() - initialYaw) * swerve.getDrivebaseRadius();

        double[] wheelRadii = new double[4];
        for (int i = 0; i < 4; i++) {
            double wheelPositionDelta = Math.abs(swerve.getWheelPositionRadians(i) - initialWheelPositions[i]);

            wheelRadii[i] = rotationalDistance / wheelPositionDelta;

            System.out.println("Module " + i + " has wheel radius " + wheelRadii[i]);
        }

        double sum = 0.0;
        for (double radius : wheelRadii)
            sum += radius;

        double averageRadius = sum / 4.0;

        System.out.println("Average radius is " + averageRadius);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(12.0);
    }
}
