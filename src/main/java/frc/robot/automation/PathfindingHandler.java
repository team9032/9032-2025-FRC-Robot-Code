package frc.robot.automation;

import static frc.robot.Constants.PathFollowingConstants.*;

import java.util.Set;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveToMovingPose;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.RotationalDriveToCoral;
import frc.robot.localization.TrackedObject.ObjectType;
import frc.robot.subsystems.swerve.KrakenSwerve;
import frc.robot.utils.ElasticUtil;
import frc.robot.utils.FieldUtil;

public class PathfindingHandler {
    private PathfindingHandler() {}

    private static Command pathToStubPath(String pathName, boolean mirror) {
        try {
            PathPlannerPath pathToFollow = PathPlannerPath.fromPathFile(pathName);
            
            if (mirror)
                pathToFollow = pathToFollow.mirrorPath();

            return AutoBuilder.pathfindThenFollowPath(pathToFollow, kDynamicPathConstraints);
        } catch (Exception e) {
            ElasticUtil.sendError("Path " + pathName + " failed to load!", "Autos will not work");

            return Commands.none();
        }
    }

    private static Pose2d getCoralAlignmentPose(KrakenSwerve swerve) {
        var optionalCoral = swerve.getLocalization().getNearestObjectOfType(ObjectType.CORAL);

        if (optionalCoral.isPresent()) {
            var robotTranslation = swerve.getLocalization().getCurrentPose().getTranslation();
            var coralTranslation = optionalCoral.get().getFieldPosition().getTranslation();

            /* Find the angle that points the robot towards the coral */
            var rotationSetpoint = robotTranslation.minus(coralTranslation).getAngle()//TODO this causes stability problems while driving if done in real time
                .plus(Rotation2d.k180deg);

            var targetPose = new Pose2d(coralTranslation, rotationSetpoint)
                /* Apply the intake's offset  */
                .transformBy(kCoralIntakeOffset);

            return targetPose;
        }

        /* If no coral is seen, maintain the current pose */
        else    
            return swerve.getLocalization().getCurrentPose();
    }

    public static Command pathToNearestCoral(KrakenSwerve swerve) {
        return Commands.defer(() -> new DriveToPose(swerve, getCoralAlignmentPose(swerve)), Set.of(swerve));
    }

    public static Command pathToNearestMovingCoral(KrakenSwerve swerve) {
        return new DriveToMovingPose(swerve, () -> getCoralAlignmentPose(swerve));
    }

    public static Command pathToBarge(KrakenSwerve swerve) {
        return Commands.defer(() -> new DriveToPose(swerve, FieldUtil.getBargeAlignmentPose(swerve.getLocalization())), Set.of(swerve));
    }

    public static Command pathToClosestReefBranch(KrakenSwerve swerve, boolean isLeftBranch) {
        return Commands.defer(() -> 
            new DriveToPose(swerve, FieldUtil.getClosestReefScoringLocation(swerve.getLocalization(), isLeftBranch)), 
            Set.of(swerve)
        );
    }

    public static Command pathToReefBranch(int reefTagID, KrakenSwerve swerve, boolean isLeftBranch) {
        return Commands.defer(() -> new DriveToPose(swerve, FieldUtil.getReefScoringLocationFromTagID(swerve.getLocalization(), isLeftBranch, reefTagID)), Set.of(swerve));
    }
    
    public static Command pathToClosestReefAlgaeIntake(KrakenSwerve swerve) {
        return Commands.defer(() -> new DriveToPose(swerve, FieldUtil.getClosestReefAlgaeIntakeLocation(swerve.getLocalization())), Set.of(swerve));
    }

    public static Command pathToSourceThenCoral(KrakenSwerve swerve, boolean isLeftSource) {
        return pathToStubPath("LSourceToCoral", !isLeftSource)
            .andThen(new RotationalDriveToCoral(swerve));
    }
}
