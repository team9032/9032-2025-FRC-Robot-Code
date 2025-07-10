package frc.robot.commands;

import static frc.robot.Constants.PathFollowingConstants.kPullAwayVelocity;
import static frc.robot.Constants.PathFollowingConstants.kRobotRelativeClosedLoopDriveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.KrakenSwerve;

public class PullAway extends Command {
    private final KrakenSwerve swerve;

    public PullAway(KrakenSwerve swerve) {
        this.swerve = swerve;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.setControl(
            kRobotRelativeClosedLoopDriveRequest.withSpeeds(new ChassisSpeeds(kPullAwayVelocity, 0, 0))
        );
    }
}