package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.KrakenSwerve;

import static frc.robot.Constants.PathplannerConfig.*;

public class AlignWithPose extends Command {
    private final ProfiledPIDController alignmentXPID;
    private final ProfiledPIDController alignmentYPID;
    private final ProfiledPIDController alignmentRotationPID;

    private final KrakenSwerve swerve;

    private final Pose2d targetPose;

    public AlignWithPose(KrakenSwerve swerve, Pose2d targetPose) {
        TrapezoidProfile.Constraints kXAlignWithPoseContraints = new TrapezoidProfile.Constraints(kDynamicPathConstraints.maxVelocityMPS(), kDynamicPathConstraints.maxAccelerationMPSSq());
        TrapezoidProfile.Constraints kYAlignWithPoseContraints = new TrapezoidProfile.Constraints(kDynamicPathConstraints.maxVelocityMPS(), kDynamicPathConstraints.maxAccelerationMPSSq());
        TrapezoidProfile.Constraints kRotAlignWithPoseContraints = new TrapezoidProfile.Constraints(kDynamicPathConstraints.maxAngularVelocityRadPerSec(), kDynamicPathConstraints.maxAngularAccelerationRadPerSecSq());

        alignmentXPID = new ProfiledPIDController(kAlignmentXYkP, 0, kAlignmentXYkD, kXAlignWithPoseContraints);
        alignmentXPID.setTolerance(kXYAlignmentTolerance);

        alignmentYPID = new ProfiledPIDController(kAlignmentXYkP, 0, kAlignmentXYkD, kYAlignWithPoseContraints);
        alignmentYPID.setTolerance(kXYAlignmentTolerance);

        alignmentRotationPID = new ProfiledPIDController(kAlignmentRotkP, 0, kAlignmentRotkD, kRotAlignWithPoseContraints);
        alignmentRotationPID.setTolerance(kRotAlignmentTolerance);
        alignmentRotationPID.enableContinuousInput(-Math.PI, Math.PI);

        this.swerve = swerve;

        this.targetPose = targetPose;      
    }

    @Override
    public void initialize() {
        Pose2d currentPose = swerve.getLocalization().getCurrentPose();

        alignmentXPID.reset(currentPose.getX());
        alignmentYPID.reset(currentPose.getY());
        alignmentRotationPID.reset(currentPose.getRotation().getRadians());

        alignmentXPID.setGoal(targetPose.getX()); 
        alignmentYPID.setGoal(targetPose.getY()); 
        alignmentRotationPID.setGoal(targetPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getLocalization().getCurrentPose();
        
        double x = alignmentXPID.calculate(currentPose.getX()) + alignmentXPID.getSetpoint().velocity;
        double y = alignmentYPID.calculate(currentPose.getY()) + alignmentYPID.getSetpoint().velocity;
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

    @Override
    public boolean isFinished() {
        return alignmentRotationPID.atGoal() && alignmentXPID.atGoal() && alignmentYPID.atGoal();
    }
}