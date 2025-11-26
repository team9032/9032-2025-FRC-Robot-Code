package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.pathing.BezierCurvePath;
import frc.robot.pathing.BezierTrajectory;
import frc.robot.subsystems.swerve.KrakenSwerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.PathFollowingConstants.kFieldCentricClosedLoopDriveRequest;
import static frc.robot.pathing.PathingConstants.*;

public class FollowBezierCurvePath extends Command {
    private final PIDController xPID;
    private final PIDController yPID;
    private final PIDController alignmentRotationPID;

    private final KrakenSwerve swerve;

    private final BezierCurvePath path;

    private BezierTrajectory trajectory;

    private final Timer timer;

    public FollowBezierCurvePath(KrakenSwerve swerve, BezierCurvePath path) {
        xPID = new PIDController(kTranslationKP, 0, kTranslationKD);
        xPID.setTolerance(kTranslationTolerance.in(Meters));

        yPID = new PIDController(kTranslationKP, 0, kTranslationKD);
        yPID.setTolerance(kTranslationTolerance.in(Meters));

        alignmentRotationPID = new PIDController(kRotationKP, 0, kRotationKD);
        alignmentRotationPID.setTolerance(kRotationTolerance.in(Radians));
        alignmentRotationPID.enableContinuousInput(-Math.PI, Math.PI);

        timer = new Timer();

        this.swerve = swerve;
        this.path = path;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        xPID.reset();
        yPID.reset();
        alignmentRotationPID.reset();

        trajectory = new BezierTrajectory(path, swerve.getLocalization().getCurrentVelocity());

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {//TODO implement acceleration?
        Pose2d currentPose = swerve.getLocalization().getCurrentPose();

        var targetState = trajectory.sampleTrajectory(timer.get());

        /* Update position PIDs */
        Pose2d targetPose = targetState.targetPose();
        xPID.setSetpoint(targetPose.getX());
        yPID.setSetpoint(targetPose.getY());
        alignmentRotationPID.setSetpoint(targetPose.getRotation().getRadians());

        /* Find speeds using PID outputs and velocity setpoints */
        double x = xPID.calculate(currentPose.getX()) + targetState.fieldCentricVelocity().getX();
        double y = yPID.calculate(currentPose.getY()) + targetState.fieldCentricVelocity().getY();
        double rot = alignmentRotationPID.calculate(currentPose.getRotation().getRadians()) + targetState.angularVelocity();

        swerve.setControl(
            kFieldCentricClosedLoopDriveRequest
                .withVelocityX(x)
                .withVelocityY(y)
                .withRotationalRate(rot)
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(
            kFieldCentricClosedLoopDriveRequest
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0.0)
        );
    }

    private boolean atSetpoint() {
        return alignmentRotationPID.atSetpoint() && xPID.atSetpoint() && yPID.atSetpoint();
    }

    @Override
    public boolean isFinished() {
        /* If there is a final speed, being at the position setpoint is not important */
        if (path.getFinalSpeed() > 0.01) {
            return timer.hasElapsed(trajectory.getTimeRequiredToFollow());
        }

        else {
            var velocity = swerve.getLocalization().getCurrentVelocity();
            boolean slowEnough = Math.hypot(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond) < kAcceptableEndingVelocity;

            return atSetpoint() && timer.hasElapsed(trajectory.getTimeRequiredToFollow()) && slowEnough;
        }
    }
}