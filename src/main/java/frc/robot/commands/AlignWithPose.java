package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.KrakenSwerve;

import static frc.robot.Constants.PathplannerConfig.*;

/** Add your docs here. */
public class AlignWithPose extends Command {
    private final ProfiledPIDController alignmentXPID;
    private final ProfiledPIDController alignmentYPID;
    private final ProfiledPIDController alignmentRotationPID;

    private final KrakenSwerve swerve;

    private final Pose2d targetPose;

    public AlignWithPose(KrakenSwerve swerve, Pose2d targetPose) {
        TrapezoidProfile.Constraints kXYAlignWithPoseContraints = new TrapezoidProfile.Constraints(kDynamicPathConstraints.maxVelocityMPS(), kDynamicPathConstraints.maxAccelerationMPSSq());
        TrapezoidProfile.Constraints kRotAlignWithPoseContraints = new TrapezoidProfile.Constraints(kDynamicPathConstraints.maxAngularVelocityRadPerSec(), kDynamicPathConstraints.maxAngularAccelerationRadPerSecSq());

        alignmentXPID = new ProfiledPIDController(kAlignmentXYkP, 0, kAlignmentXYkD, kXYAlignWithPoseContraints);
        alignmentXPID.setTolerance(kXYAlignmentTolerance);

        alignmentYPID = new ProfiledPIDController(kAlignmentXYkP, 0, kAlignmentXYkD, kXYAlignWithPoseContraints);
        alignmentYPID.setTolerance(kXYAlignmentTolerance);

        alignmentRotationPID = new ProfiledPIDController(kAlignmentRotkP, 0, kAlignmentRotkD, kRotAlignWithPoseContraints);
        alignmentRotationPID.setTolerance(kRotAlignmentTolerance);

        this.swerve = swerve;

        this.targetPose = targetPose;
    }

    @Override
    public void initialize() {
        alignmentXPID.setGoal(targetPose.getX()); 
        alignmentYPID.setGoal(targetPose.getY()); 
        alignmentRotationPID.setGoal(targetPose.getRotation().getRadians());
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
        alignmentXPID.reset(new TrapezoidProfile.State());
        alignmentYPID.reset(new TrapezoidProfile.State());
        alignmentRotationPID.reset(new TrapezoidProfile.State());
    }

    @Override
    public boolean isFinished() {
        return alignmentXPID.atSetpoint() && alignmentYPID.atSetpoint() && alignmentRotationPID.atSetpoint();
    }
}