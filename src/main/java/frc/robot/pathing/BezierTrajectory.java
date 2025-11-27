package frc.robot.pathing;

import static frc.robot.pathing.PathingConstants.kCurveSampleAmount;
import static frc.robot.pathing.PathingConstants.kDefaultRotationConstraints;
import static frc.robot.pathing.PathingConstants.kDefaultTranslationConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

public class BezierTrajectory {
    private final BezierCurvePath path;

    private TrapezoidProfile translationProfile;
    private final State initialTranslationState;
    private final State finalTranslationState;

    private final TrapezoidProfile rotationProfile;
    private final State initialRotationState;
    private final State finalRotationState;

    private double timeToFollow;

    private final StructArrayPublisher<Pose2d> trajectoryPublisher;

    public BezierTrajectory(BezierCurvePath path, ChassisSpeeds initialSpeeds) {
        this(path, initialSpeeds, kDefaultTranslationConstraints, kDefaultRotationConstraints);
    }

    public BezierTrajectory(BezierCurvePath path, ChassisSpeeds initialSpeeds, Constraints translationConstraints, Constraints rotationConstraints) {
        this.path = path;

        translationProfile = new TrapezoidProfile(translationConstraints);
        rotationProfile = new TrapezoidProfile(rotationConstraints);

        //TODO handle initial velocity not in direction of path
        /* Setup translation profile */
        initialTranslationState = new State(0.0, Math.hypot(initialSpeeds.vxMetersPerSecond, initialSpeeds.vyMetersPerSecond));
        finalTranslationState = new State(path.getLength(), path.getFinalSpeed());
        translationProfile.calculate(0, initialTranslationState, finalTranslationState);
        double translationTime = translationProfile.timeLeftUntil(finalTranslationState.position);        

        /* Setup rotation profile *///TODO rotation is broken
        initialRotationState = new State(path.getInitialRotation().getRadians(), initialSpeeds.omegaRadiansPerSecond);
        finalRotationState = new State(path.getFinalRotation().getRadians(), 0.0);
        rotationProfile.calculate(0, initialRotationState, finalRotationState);
        double rotationTime = rotationProfile.timeLeftUntil(path.getFinalRotation().getRadians());

        /* If needed, slow down to allow the rotation profile to finish before the end of the trajectory */
        if (rotationTime > translationTime) {
            // translationProfile = new TrapezoidProfile(translationConstraints);//TODO implement

            // translationTime = rotationTime;
        }

        timeToFollow = translationTime;

        trajectoryPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Bezier Trajectory", Pose2d.struct)
            .publish();
    }

    public BezierTrajectoryState sampleTrajectory(double time) {
        var translationProfileState = translationProfile.calculate(time, initialTranslationState, finalTranslationState);

        var pathPose = path.samplePathPosition(translationProfileState.position);
        var pathDirection = path.samplePathDirection(translationProfileState.position);
        var pathVelocity = pathDirection.div(pathDirection.getNorm()).times(translationProfileState.velocity);

        var rotationProfileState = rotationProfile.calculate(time, initialRotationState, finalRotationState);

        return new BezierTrajectoryState(
            new Pose2d(pathPose, new Rotation2d(rotationProfileState.position)), 
            pathVelocity, 
            null,//TODO handle acceleration 
            rotationProfileState.velocity
        );
    }

    public double getTimeRequiredToFollow() {
        return timeToFollow;
    }

    public void graphTrajectory() {
        Pose2d[] poses = new Pose2d[kCurveSampleAmount];
        for (int i = 0; i < kCurveSampleAmount; i++) {
            double time = ((double) i / kCurveSampleAmount) * getTimeRequiredToFollow();

            poses[i] = sampleTrajectory(time).targetPose();
        }

        trajectoryPublisher.accept(poses);
    }
}