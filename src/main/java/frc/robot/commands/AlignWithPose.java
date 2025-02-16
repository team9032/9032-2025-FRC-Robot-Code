package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.KrakenSwerve;

import static frc.robot.Constants.PathplannerConfig.*;

/** Add your docs here. */
public class AlignWithPose extends Command {
    private final PIDController alignmentXPID;
    private final PIDController alignmentYPID;
    private final PIDController alignmentRotationPID;

    private final KrakenSwerve swerve;

    private final Pose2d targetPose;

    public AlignWithPose(KrakenSwerve swerve, Pose2d targetPose) {
        alignmentXPID = new PIDController(kAlignmentXYkP, 0, kAlignmentXYkD);
        alignmentXPID.setTolerance(kXYAlignmentTolerance);

        alignmentYPID = new PIDController(kAlignmentXYkP, 0, kAlignmentXYkD);
        alignmentYPID.setTolerance(kXYAlignmentTolerance);

        alignmentRotationPID = new PIDController(kAlignmentRotkP, 0, kAlignmentRotkD);
        alignmentRotationPID.setTolerance(kRotAlignmentTolerance);

        this.swerve = swerve;

        this.targetPose = targetPose;
    }

    @Override
    public void initialize() {
        alignmentXPID.setSetpoint(targetPose.getX()); 
        alignmentYPID.setSetpoint(targetPose.getY()); 
        alignmentRotationPID.setSetpoint(targetPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.drivetrain.getState().Pose;

        double x = alignmentXPID.calculate(currentPose.getX());
        double y = alignmentYPID.calculate(currentPose.getY());
        double rot = -alignmentRotationPID.calculate(currentPose.getRotation().getRadians());

        SmartDashboard.putNumber("X Alignment ", x);//TODO remove these telemtery
        SmartDashboard.putNumber("Y Alignment ", y);
        SmartDashboard.putNumber("Rot ", rot);

        SmartDashboard.putString("Current", currentPose.toString());
        SmartDashboard.putString("Target", targetPose.toString());

        ChassisSpeeds speeds = new ChassisSpeeds(x, y, rot);

        swerve.drivetrain.setControl(
            kClosedLoopDriveRequest.withSpeeds(speeds)
        );
    }

    @Override
    public void end(boolean interrupted) {
        alignmentXPID.reset();
        alignmentYPID.reset();
        alignmentRotationPID.reset();
    }

    @Override
    public boolean isFinished() {
        return alignmentXPID.atSetpoint() && alignmentYPID.atSetpoint() && alignmentRotationPID.atSetpoint();
    }
}
