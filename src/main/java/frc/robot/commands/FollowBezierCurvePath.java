package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.pathing.BezierCurvePath;
import frc.robot.subsystems.swerve.KrakenSwerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.PathFollowingConstants.*;

public class FollowBezierCurvePath extends Command {
    private final PIDController alignmentXPID;
    private final PIDController alignmentYPID;
    private final ProfiledPIDController alignmentRotationPID;

    private final KrakenSwerve swerve;

    private final BezierCurvePath path;

    public FollowBezierCurvePath(KrakenSwerve swerve, BezierCurvePath path) {
        alignmentXPID = new PIDController(kAlignmentXYkP, 0, kAlignmentXYkD);
        alignmentXPID.setTolerance(kXYAlignmentTolerance.in(Meters));

        alignmentYPID = new PIDController(kAlignmentXYkP, 0, kAlignmentXYkD);
        alignmentYPID.setTolerance(kXYAlignmentTolerance.in(Meters));

        alignmentRotationPID = new ProfiledPIDController(kAlignmentRotkP, 0, kAlignmentRotkD, kDriveToPoseRotationConstraints);
        alignmentRotationPID.setTolerance(kRotAlignmentTolerance.in(Radians));
        alignmentRotationPID.enableContinuousInput(-Math.PI, Math.PI);

        this.swerve = swerve;

        this.path = path;      

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = swerve.getLocalization().getCurrentPose();
        ChassisSpeeds currentVelocity = swerve.getLocalization().getCurrentVelocity();

        alignmentXPID.reset();
        alignmentYPID.reset();
        alignmentRotationPID.reset(currentPose.getRotation().getRadians(), currentVelocity.omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getLocalization().getCurrentPose();

        double speedSetpoint = 

        double x = alignmentXPID.calculate(currentPose.getX());
        double y = alignmentYPID.calculate(currentPose.getY());
        double rot = alignmentRotationPID.calculate(currentPose.getRotation().getRadians()) + alignmentRotationPID.getSetpoint().velocity;

        swerve.setControl(
            kFieldCentricClosedLoopDriveRequest
                .withVelocityX(x)
                .withVelocityY(y)
                .withRotationalRate(rot)
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(kRobotRelativeClosedLoopDriveRequest.withSpeeds(new ChassisSpeeds()));
    }

    private boolean atGoal() {
        return alignmentRotationPID.atGoal() && alignmentXPID.atSetpoint() && alignmentYPID.atSetpoint();
    }

    @Override
    public boolean isFinished() {
        return atGoal();
    }
}