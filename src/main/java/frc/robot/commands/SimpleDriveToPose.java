package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.KrakenSwerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import static frc.robot.pathing.PathingConstants.*;

public class SimpleDriveToPose extends Command {
    private final PIDController alignmentXPID;
    private final PIDController alignmentYPID;
    private final ProfiledPIDController alignmentRotationPID;

    private final KrakenSwerve swerve;

    private final Pose2d targetPose;

    public SimpleDriveToPose(KrakenSwerve swerve, Pose2d targetPose) {
        alignmentXPID = new PIDController(kTranslationkP, 0, kTranslationkD);
        alignmentXPID.setTolerance(kXYAlignmentTolerance.in(Meters));

        alignmentYPID = new PIDController(kTranslationkP, 0, kTranslationkD);
        alignmentYPID.setTolerance(kXYAlignmentTolerance.in(Meters));

        alignmentRotationPID = new ProfiledPIDController(kRotationkP, 0, kRotationkD, kRotationConstraints);
        alignmentRotationPID.setTolerance(kRotationAlignmentTolerance.in(Radians));
        alignmentRotationPID.enableContinuousInput(-Math.PI, Math.PI);

        this.swerve = swerve;

        this.targetPose = targetPose;      

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = swerve.getLocalization().getCurrentPose();
        ChassisSpeeds currentVelocity = swerve.getLocalization().getCurrentVelocity();

        alignmentXPID.setSetpoint(targetPose.getX()); 
        alignmentYPID.setSetpoint(targetPose.getY()); 
        alignmentRotationPID.setGoal(targetPose.getRotation().getRadians());

        alignmentXPID.reset();
        alignmentYPID.reset();
        alignmentRotationPID.reset(currentPose.getRotation().getRadians(), currentVelocity.omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getLocalization().getCurrentPose();

        double x = alignmentXPID.calculate(currentPose.getX());
        double y = alignmentYPID.calculate(currentPose.getY());
        double rot = alignmentRotationPID.calculate(currentPose.getRotation().getRadians()) + alignmentRotationPID.getSetpoint().velocity;

        swerve.setControl(
            kFieldCentricClosedLoopDriveRequest
                .withVelocityX(x)
                .withVelocityY(y)
                .withRotationalRate(rot)
        );

        SmartDashboard.putBoolean("Simple Drive To Pose At Goal", atGoal());
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