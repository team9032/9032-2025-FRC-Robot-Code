package frc.robot.simulation;

import org.ironmaple.simulation.SimulatedArena;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

public class SimulationControls {
    private static final StructArrayPublisher<Pose3d> coralPosePub = NetworkTableInstance.getDefault()
      .getTable("Simulation")
      .getStructArrayTopic("Coral Poses", Pose3d.struct)
      .publish();

    private static final StructArrayPublisher<Pose3d> algaePosePub = NetworkTableInstance.getDefault()
      .getTable("Simulation")
      .getStructArrayTopic("Algae Poses", Pose3d.struct)
      .publish();

    public static void init() {
        SimulatedArena.getInstance().placeGamePiecesOnField();
    }

    public static void update() {
        SimulatedArena.getInstance().simulationPeriodic();

        var coralPoses = SimulatedArena.getInstance()
            .getGamePiecesArrayByType("Coral");

        var algaePoses = SimulatedArena.getInstance()
            .getGamePiecesArrayByType("Algae");

        coralPosePub.set(coralPoses);
        algaePosePub.set(algaePoses);
    }
}
