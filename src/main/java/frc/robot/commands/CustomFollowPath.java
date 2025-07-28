package frc.robot.commands;

import static frc.robot.Constants.PathFollowingConstants.kRobotRelativeClosedLoopDriveRequest;
import static frc.robot.Constants.PathFollowingConstants.kRotAlignmentTolerance;
import static frc.robot.Constants.PathFollowingConstants.kRotationPID;
import static frc.robot.Constants.PathFollowingConstants.kTranslationPID;
import static frc.robot.Constants.PathFollowingConstants.kXYAlignmentTolerance;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.KrakenSwerve;
import frc.robot.utils.FieldUtil;

public class CustomFollowPath extends Command {
    private final KrakenSwerve swerve;
    private final PPHolonomicDriveController driveController;
    private final Timer timer;

    private PathPlannerPath pathToFollow;

    private PathPlannerTrajectory trajectory;

    private Pose2d endPose;

    public CustomFollowPath(KrakenSwerve swerve, PathPlannerPath pathToFollow) {
        this.swerve = swerve;
        this.pathToFollow = pathToFollow;
        
        driveController = new PPHolonomicDriveController(kTranslationPID, kRotationPID);
        timer = new Timer();

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        /* Flip on init to make sure the driver station is connected */
        if (FieldUtil.shouldFlipCoordinates() && !pathToFollow.preventFlipping)
            pathToFollow = pathToFollow.flipPath();

        /* Find end pose */
        var goalTranslation = pathToFollow.getWaypoints().get(pathToFollow.getWaypoints().size() - 1).anchor();
        var goalRotation = pathToFollow.getGoalEndState().rotation();
        endPose = new Pose2d(goalTranslation, goalRotation);

        var currentSpeeds = swerve.getLocalization().getCurrentVelocity();
        var currentPose = swerve.getLocalization().getCurrentPose();

        trajectory = pathToFollow.generateTrajectory(
            currentSpeeds, 
            currentPose.getRotation(), 
            swerve.getPathplannerConfig()
        );

        driveController.reset(currentPose, currentSpeeds);

        timer.start();
    }

    @Override
    public void execute() {
        var targetState = trajectory.sample(timer.get());

        var targetSpeeds = driveController.calculateRobotRelativeSpeeds(swerve.getLocalization().getCurrentPose(), targetState);

        swerve.drivePathPlanner(targetSpeeds, targetState.feedforwards);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();

        if (pathToFollow.getGoalEndState().velocityMPS() < 0.01 || interrupted)
            swerve.setControl(kRobotRelativeClosedLoopDriveRequest.withSpeeds(new ChassisSpeeds()));
    }

    @Override
    public boolean isFinished() {
        /* Only end when the trajectory is finished */
        if (!timer.hasElapsed(trajectory.getTotalTimeSeconds()))
            return false;

        /* For non zero ending velocities, finish when the trajectory ends to avoid driving back */
        if (pathToFollow.getGoalEndState().velocityMPS() > 0.01)
            return true;

        /* End when within tolerance */
        Pose2d currentPose = swerve.getLocalization().getCurrentPose();

        return endPose.getTranslation().getDistance(currentPose.getTranslation()) < kXYAlignmentTolerance//TODO different constants
            && MathUtil.isNear(endPose.getRotation().getRadians(), currentPose.getRotation().getRadians(), kRotAlignmentTolerance);
    }
}
