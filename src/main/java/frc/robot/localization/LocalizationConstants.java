package frc.robot.localization;

import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class LocalizationConstants {
    /* Constants for the confidence calculator */
    public static final double kPoseAmbiguityMultiplier = 40;
    public static final double kNoisyDistanceMeters = 2.5;
    public static final double kDistanceWeight = 7;

    /* Thresholds for when to reject an estimate */
    public static final double kAmbiguityThreshold = 0.2;
    public static final double kDistanceThreshold = 4.0;//Meters

    /* Object tracking constants */
    public static final double kObjectExpireTime = 0.5;//Seconds
    public static final double kSameObjectDistance = Units.inchesToMeters(6);
    public static final int kExcessiveObjectAmount = 20;

    /* Class Ids */
    public static final int kCoralId = 1;
    public static final int kAlgaeId = 0;

    /* Object heights */
    public static final double kCoralHeight = Units.inchesToMeters(4.5);
    public static final double kAlgaeHeight = Units.inchesToMeters(16.25);

    public static final double kPoseLookaheadTime = 0.15;//Seconds
    
    public static final Matrix<N3, N1> kSingleTagBaseStandardDeviations = VecBuilder.fill(
        1,//X
        1,//Y
        1 * Math.PI//Theta
    );

    public static final Matrix<N3, N1> kMultiTagBaseStandardDeviations = VecBuilder.fill(
        0.25,//X
        0.25,//Y
        0.5 * Math.PI//Theta
    );

    public static final String kAprilTagFieldLayoutName = "2025-reefscape.json";//Loads from a JSON file in deploy

    public static final CameraConstants[] kCameraConstants = new CameraConstants[] {
        new CameraConstants("FrontCenterCamera", new Transform3d(
            new Translation3d(Units.inchesToMeters(8), Units.inchesToMeters(-5.5), Units.inchesToMeters(31.125)), 
            new Rotation3d(0, Units.degreesToRadians(30), 0)),
            true
        ),
        new CameraConstants("BackLeftCamera", new Transform3d(
           new Translation3d(Units.inchesToMeters(3.5625), Units.inchesToMeters(14.25), Units.inchesToMeters(7.8125)), 
           new Rotation3d(0,0, Units.degreesToRadians(150))),
           false
        ),
        new CameraConstants("BackCenterCamera", new Transform3d(
           new Translation3d(Units.inchesToMeters(-14.375),Units.inchesToMeters(6.0),Units.inchesToMeters(6.75)),
           new Rotation3d(0,Units.degreesToRadians(-20),Math.PI)),
           false
        ),
        new CameraConstants("BackRightCamera", new Transform3d(
            new Translation3d(Units.inchesToMeters(-3.25),Units.inchesToMeters(-13.8125),Units.inchesToMeters(11.625)),
            new Rotation3d(0,Units.degreesToRadians(-10),Math.PI)),
            false
        )
    };

    /* Field constants from the game manual */
    public static final Translation2d kReefCenter = new Translation2d(Units.inchesToMeters(176.746), 8.052 / 2.0);
    public static final TargetModel kCoralModel = new TargetModel(Units.inchesToMeters(11.875), Units.inchesToMeters(4.5), Units.inchesToMeters(4.5));
    public static final int kMinReefTagID = 17;
    public static final int kMaxReefTagID = 22;
    public static final int kBackReefTagsStartingID = 20;

    /* Simulation camera properties */
    public static final SimCameraProperties kObjectTrackingSimCameraProperties = new SimCameraProperties()
        .setCalibration(800, 600, Rotation2d.fromDegrees(70))
        .setCalibError(0.35, 0.10)
        .setFPS(30)
        .setAvgLatencyMs(20)
        .setLatencyStdDevMs(5);

    public static final SimCameraProperties kLocalizationSimCameraProperties = new SimCameraProperties()
        .setCalibration(800, 600, Rotation2d.fromDegrees(60))
        .setCalibError(0.35, 0.10)
        .setFPS(30)
        .setAvgLatencyMs(20)
        .setLatencyStdDevMs(5);
}
