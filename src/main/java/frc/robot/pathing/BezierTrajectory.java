package frc.robot.pathing;

import java.util.TreeMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class BezierTrajectory {
    private final BezierCurvePath path;
    private final TrapezoidProfile profile;
    private final State initialState;
    private final State finalState;

    private final TreeMap<Double, BezierTrajectoryState> timeToStateMap = new TreeMap<>();

    private final double kTotalStates = 20;

    public BezierTrajectory(BezierCurvePath path) {
        this(path, new State(0.0, 0.0), new State(0.0, 0.0));
    }

    public BezierTrajectory(BezierCurvePath path, State initialState, State finalState) {
        this.path = path;
        this.initialState = initialState;
        this.finalState = finalState;

        profile = new TrapezoidProfile(new Constraints(4, 5));//TODO constants

        var initialProfileState = profile.calculate(0, initialState, finalState);//TODO how else to set current state?
        //TODO add initial profile state
        double totalTime = profile.timeLeftUntil(finalState.position);        

        for (int i = 1; i < kTotalStates; i++) {
            double time = totalTime * (i / kTotalStates);

            var profileState = profile.calculate(time, initialState, finalState);

            var pathPose = path.samplePathPosition(path.getTimeFromDistance(profileState.position));
            var pathDirection = path.samplePathDirection(profileState.position);
            var pathVelocity = pathDirection.div(pathDirection.getNorm()).times(profileState.velocity);

            timeToStateMap.put(
                time, 
                new BezierTrajectoryState(new Pose2d(pathPose, Rotation2d.kZero), pathVelocity, null)
            );
        }
    }

    public BezierTrajectoryState sampleTrajectory(double time) {
        //TODO
    }
}