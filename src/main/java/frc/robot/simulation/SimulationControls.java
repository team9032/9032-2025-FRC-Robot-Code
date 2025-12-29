package frc.robot.simulation;

import static edu.wpi.first.units.Units.Seconds;

import org.ironmaple.simulation.SimulatedArena;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import frc.robot.subsystems.swerve.KrakenSwerve;

public class SimulationControls {
    private static final StructArrayPublisher<Pose3d> coralPosePub = NetworkTableInstance.getDefault()
      .getTable("Simulation")
      .getStructArrayTopic("Coral Poses", Pose3d.struct)
      .publish();

    private static final StructArrayPublisher<Pose3d> algaePosePub = NetworkTableInstance.getDefault()
      .getTable("Simulation")
      .getStructArrayTopic("Algae Poses", Pose3d.struct)
      .publish();

    public static void init(double simulationPeriod) {
        SimulatedArena.overrideSimulationTimings(Seconds.of(simulationPeriod), 1);
        SimulatedArena.getInstance().placeGamePiecesOnField();
    }

    public static void update(KrakenSwerve swerve) {
        SimulatedArena.getInstance().simulationPeriodic();
        swerve.updateDrivetrainSimulation();

        var coralPoses = SimulatedArena.getInstance()
            .getGamePiecesArrayByType("Coral");

        var algaePoses = SimulatedArena.getInstance()
            .getGamePiecesArrayByType("Algae");

        coralPosePub.set(coralPoses);
        algaePosePub.set(algaePoses);
    }
}
