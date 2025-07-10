// package frc.robot.commands;

// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.networktables.StructPublisher;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.swerve.KrakenSwerve;

// import static frc.robot.Constants.PathFollowingConstants.*;

// import java.util.function.Supplier;

// public class StraightDriveToPose extends Command {
//     private final ProfiledPIDController alignmentTranslationPID;
//     private final ProfiledPIDController alignmentRotationPID;

//     private final KrakenSwerve swerve;

//     private final Supplier<Pose2d> targetPoseSup;

//     private final StructPublisher<Pose2d> setpointPublisher;

//     public StraightDriveToPose(KrakenSwerve swerve, Supplier<Pose2d> targetPoseSup) {
//         alignmentTranslationPID = new ProfiledPIDController(kAlignmentXYkP, 0, kAlignmentXYkD, kDriveToPoseTranslationConstraints);
//         alignmentTranslationPID.setTolerance(kXYAlignmentTolerance);

//         alignmentRotationPID = new ProfiledPIDController(kAlignmentRotkP, 0, kAlignmentRotkD, kDriveToPoseRotationConstraints);
//         alignmentRotationPID.setTolerance(kRotAlignmentTolerance);
//         alignmentRotationPID.enableContinuousInput(-Math.PI, Math.PI);

//         this.swerve = swerve;

//         this.targetPoseSup = targetPoseSup;      

//         setpointPublisher = NetworkTableInstance.getDefault()
//             .getStructTopic("Straight drive to pose setpoint", Pose2d.struct)
//             .publish();
//     }

//     @Override
//     public void initialize() {
//         Pose2d currentPose = swerve.getLocalization().getCurrentPose();
//         ChassisSpeeds currentVelocity = swerve.getLocalization().getCurrentVelocity();

//         alignmentTranslationPID.reset(currentPose.getX(), currentVelocity.);
//         alignmentRotationPID.reset(currentPose.getRotation().getRadians(), currentVelocity.omegaRadiansPerSecond);
//     }

//     @Override
//     public void execute() {
//         updateControllerGoals();

//         Pose2d currentPose = swerve.getLocalization().getCurrentPose();

//         double x = alignmentTranslationPID.calculate(currentPose.getX()) + alignmentTranslationPID.getSetpoint().velocity;
//         double y = alignmentYPID.calculate(currentPose.getY()) + alignmentYPID.getSetpoint().velocity;
//         double rot = alignmentRotationPID.calculate(currentPose.getRotation().getRadians()) + alignmentRotationPID.getSetpoint().velocity;

//         swerve.setControl(
//             kFieldCentricClosedLoopDriveRequest
//                 .withVelocityX(x)
//                 .withVelocityY(y)
//                 .withRotationalRate(rot)
//         );

//         /* Publish the motion profile's setpoint for tuning and debugging purposes */
//         setpointPublisher.set(
//             new Pose2d(alignmentTranslationPID.getSetpoint().position, alignmentYPID.getSetpoint().position, Rotation2d.fromRadians(alignmentRotationPID.getSetpoint().position))
//         );

//         SmartDashboard.putBoolean("Drive To Moving Pose At Goal", atGoal());
//     }

//     private void updateControllerGoals() {
//         alignmentTranslationPID.setGoal(targetPoseSup.get().getX()); 
//         alignmentYPID.setGoal(targetPoseSup.get().getY()); 
//         alignmentRotationPID.setGoal(targetPoseSup.get().getRotation().getRadians());
//     }

//     @Override
//     public void end(boolean interrupted) {
//         swerve.setControl(kRobotRelativeClosedLoopDriveRequest.withSpeeds(new ChassisSpeeds()));
//     }

//     private boolean atGoal() {
//         return alignmentRotationPID.atGoal() && alignmentTranslationPID.atGoal() && alignmentYPID.atGoal();
//     }

//     @Override
//     public boolean isFinished() {
//         return atGoal();
//     }
// }