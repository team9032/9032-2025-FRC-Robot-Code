package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.KrakenSwerve;

import static frc.robot.Constants.PathplannerConfig.*;

public class AlignWithPose extends Command {
    private final PIDController alignmentXPID;
    private final PIDController alignmentYPID;
    private final ProfiledPIDController alignmentRotationPID;

    private final KrakenSwerve swerve;

    private final Pose2d targetPose;

    public AlignWithPose(KrakenSwerve swerve, Pose2d targetPose) {
        // TrapezoidProfile.Constraints kXAlignWithPoseContraints = new TrapezoidProfile.Constraints(kDynamicPathConstraints.maxVelocityMPS(), kDynamicPathConstraints.maxAccelerationMPSSq());
        // TrapezoidProfile.Constraints kYAlignWithPoseContraints = new TrapezoidProfile.Constraints(kDynamicPathConstraints.maxVelocityMPS(), kDynamicPathConstraints.maxAccelerationMPSSq());
        TrapezoidProfile.Constraints kRotAlignWithPoseContraints = new TrapezoidProfile.Constraints(kDynamicPathConstraints.maxAngularVelocityRadPerSec(), kDynamicPathConstraints.maxAngularAccelerationRadPerSecSq());

        alignmentXPID = new PIDController(kAlignmentXYkP, 0, kAlignmentXYkD);
        alignmentXPID.setTolerance(kXYAlignmentTolerance);

        alignmentYPID = new PIDController(kAlignmentXYkP, 0, kAlignmentXYkD);
        alignmentYPID.setTolerance(kXYAlignmentTolerance);

        alignmentRotationPID = new ProfiledPIDController(kAlignmentRotkP, 0, kAlignmentRotkD, kRotAlignWithPoseContraints);
        alignmentRotationPID.setTolerance(kRotAlignmentTolerance);
        alignmentRotationPID.enableContinuousInput(-Math.PI, Math.PI);

        this.swerve = swerve;

        this.targetPose = targetPose;      
    }

    @Override
    public void initialize() {
        Pose2d currentPose = swerve.drivetrain.getState().Pose;

        alignmentXPID.reset();
        alignmentYPID.reset();
        alignmentRotationPID.reset(currentPose.getRotation().getRadians());

        alignmentXPID.setSetpoint(targetPose.getX()); 
        alignmentYPID.setSetpoint(targetPose.getY()); 
        alignmentRotationPID.setGoal(targetPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.drivetrain.getState().Pose;
        
        double x = alignmentXPID.calculate(currentPose.getX());
        double y = alignmentYPID.calculate(currentPose.getY());
        double rot = alignmentRotationPID.calculate(currentPose.getRotation().getRadians());//TODO this won't work

        ChassisSpeeds speeds = new ChassisSpeeds(x, y, rot);

        swerve.drivetrain.setControl(
            kClosedLoopDriveRequest.withSpeeds(speeds)
        );
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return alignmentRotationPID.atGoal() && alignmentXPID.atSetpoint() && alignmentYPID.atSetpoint();
    }
}