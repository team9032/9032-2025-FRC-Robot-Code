package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.KrakenSwerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.PathFollowingConstants.*;

import java.util.function.Supplier;

public class DriveToMovingPose extends Command {
    private final PIDController alignmentXPID;
    private final PIDController alignmentYPID;
    private final ProfiledPIDController alignmentRotationPID;

    private final KrakenSwerve swerve;

    private final Supplier<Pose2d> targetPoseSup;

    private final StructPublisher<Pose2d> setpointPublisher;

    public DriveToMovingPose(KrakenSwerve swerve, Supplier<Pose2d> targetPoseSup) {
        alignmentXPID = new PIDController(kAlignmentXYkP, 0, kAlignmentXYkD);
        alignmentXPID.setTolerance(kXYAlignmentTolerance.in(Meters));

        alignmentYPID = new PIDController(kAlignmentXYkP, 0, kAlignmentXYkD);
        alignmentYPID.setTolerance(kXYAlignmentTolerance.in(Meters));

        alignmentRotationPID = new ProfiledPIDController(kAlignmentRotkP, 0, kAlignmentRotkD, kDriveToPoseRotationConstraints);
        alignmentRotationPID.setTolerance(kRotAlignmentTolerance.in(Radians));
        alignmentRotationPID.enableContinuousInput(-Math.PI, Math.PI);

        this.swerve = swerve;

        this.targetPoseSup = targetPoseSup;      

        setpointPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Drive to moving pose setpoint", Pose2d.struct)
            .publish();

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
        updateControllerGoals();

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

        // /* Publish the motion profile's setpoint for tuning and debugging purposes */
        // setpointPublisher.set(
        //     new Pose2d(alignmentXPID.getSetpoint().position, alignmentYPID.getSetpoint().position, Rotation2d.fromRadians(alignmentRotationPID.getSetpoint().position))
        // );

        SmartDashboard.putBoolean("Drive To Moving Pose At Goal", atGoal());
    }

    private void updateControllerGoals() {
        alignmentXPID.setSetpoint(targetPoseSup.get().getX()); 
        alignmentYPID.setSetpoint(targetPoseSup.get().getY()); 
        alignmentRotationPID.setGoal(targetPoseSup.get().getRotation().getRadians());
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