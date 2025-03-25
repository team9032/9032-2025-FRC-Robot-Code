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

    //private Field2d field = new Field2d();

    public AlignWithPose(KrakenSwerve swerve, Pose2d targetPose) {
        TrapezoidProfile.Constraints kXYAlignWithPoseContraints = new TrapezoidProfile.Constraints(kDynamicPathConstraints.maxVelocityMPS(), kDynamicPathConstraints.maxAccelerationMPSSq());
        TrapezoidProfile.Constraints kRotAlignWithPoseContraints = new TrapezoidProfile.Constraints(kDynamicPathConstraints.maxAngularVelocityRadPerSec(), kDynamicPathConstraints.maxAngularAccelerationRadPerSecSq());

        alignmentXPID = new ProfiledPIDController(kAlignmentXYkP, 0, kAlignmentXYkD, kXYAlignWithPoseContraints);
        alignmentXPID.setTolerance(kXYAlignmentTolerance);

        alignmentYPID = new ProfiledPIDController(kAlignmentXYkP, 0, kAlignmentXYkD, kXYAlignWithPoseContraints);
        alignmentYPID.setTolerance(kXYAlignmentTolerance);

        alignmentRotationPID = new ProfiledPIDController(kAlignmentRotkP, 0, kAlignmentRotkD, kRotAlignWithPoseContraints);
        alignmentRotationPID.setTolerance(kRotAlignmentTolerance);
        alignmentRotationPID.enableContinuousInput(-Math.PI, Math.PI);

        this.swerve = swerve;

        this.targetPose = targetPose;      

        // SmartDashboard.putData("The field",  field);//TODO this can be removed later
    }

    @Override
    public void initialize() {
        Pose2d currentPose = swerve.drivetrain.getState().Pose;

        alignmentXPID.reset(currentPose.getX());
        alignmentYPID.reset(currentPose.getY());
        alignmentRotationPID.reset(currentPose.getRotation().getRadians());

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

        // field.getObject("Alignment Target").setPose(targetPose);
        // field.getObject("Setpoint pose").setPose(new Pose2d(alignmentXPID.getSetpoint().position, alignmentYPID.getSetpoint().position, Rotation2d.fromRadians(alignmentRotationPID.getSetpoint().position)));

        ChassisSpeeds speeds = new ChassisSpeeds(x, y, rot);

        swerve.drivetrain.setControl(
            kClosedLoopDriveRequest.withSpeeds(speeds)
        );
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return alignmentRotationPID.atGoal() && alignmentXPID.atGoal() && alignmentYPID.atGoal();
    }
}