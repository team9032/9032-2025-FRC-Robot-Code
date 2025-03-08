package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.localization.CameraConstants;
import frc.robot.subsystems.swerve.SwerveConstants;

public final class Constants {
    public static class DriverConstants {
        public static final boolean kRunSysId = false;

        public static final int kDriveControllerPort = 0;

        public static final double kMaxSpeed = SwerveConstants.kSpeedAt12Volts.magnitude();
        public static final double kRotationRate = 4 * Math.PI;

        public final static FieldCentric kDriveRequest = new FieldCentric()
            .withDeadband(kMaxSpeed * 0.01) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    }

    public static class PathplannerConfig {
        public static final PIDConstants kTranslationPID = new PIDConstants(5.0);//TODO Tune
        public static final PIDConstants kRotationPID = new PIDConstants(5.0);

        public static final ApplyRobotSpeeds kClosedLoopDriveRequest = new ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

        public static final PathConstraints kDynamicPathConstraints = new PathConstraints(
            4,//TODO change these
            4, 
            2 * Math.PI, 
            2 * Math.PI
        );

        public static final double kAlignmentXYkP = 2;//TODO Tune better
        public static final double kAlignmentXYkD = 0;
        
        public static final double kAlignmentRotkP = 10.0;
        public static final double kAlignmentRotkD = 0;

        public static final double kXYAlignmentTolerance = Units.inchesToMeters(0.5);
        public static final double kRotAlignmentTolerance = Units.degreesToRadians(1);
    }

    public static final class ObjectAimingConstants {
        public static final String kObjectTrackingCameraName = "FrontCenterCamera";

        public static final int kCycleAmtSinceTargetSeenCutoff = 10;
        public static final double kPitchDifferenceCutoff = 2.0;

        public static final double kRotationSetpoint = 18.8;
        //TODO make faster
        public static final double kDrivingSpeed = 2.0;//Meters per second

        /* PID Constants */
        public static final double kPRotation = 0.15;
        public static final double kDRotation = 0.002;

        /* Class Ids */
        public static final int kCoralId = 1;
        public static final int kAlgaeId = 0;
    }

    public static final class LocalizationConstants {//TODO all under need to be tuned 
        /* Constants for the confidence calculator */
        public static final double kPoseAmbiguityMultiplier = 40;
        public static final double kNoisyDistanceMeters = 2.5;
        public static final double kDistanceWeight = 7;

        /* Thresholds for when to reject an estimate */
        public static final double kAmbiguityThreshold = 0.2;
        public static final double kDistanceThreshold = 4.0;//Meters
        
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

        /* Used for cameras mounted on swerve modules */
        public static final double kCameraHeight = Units.inchesToMeters(8.0);//Height off of the ground in meters
        public static final double kCameraPitch = Units.degreesToRadians(17.0);//The camera's pitch

        public static final double kXandYCoord = 0.282575;

        public static final CameraConstants[] kCameraConstants = new CameraConstants[] {
            // new CameraConstants("FrontCenterCamera", new Transform3d(
            //     new Translation3d(0, 0, 0), //TODO find offsets for this camera
            //     new Rotation3d(0, kCameraPitch,  -(Math.PI / 4.0))),
            //     true
            // ),
            // new CameraConstants("FrontRightCamera", new Transform3d(
            //     new Translation3d(kXandYCoord, -kXandYCoord, kCameraHeight), 
            //     new Rotation3d(0, kCameraPitch,  -(Math.PI / 4.0))),
            //     false
            // ),
            // new CameraConstants("FrontLeftCamera", new Transform3d(
            //     new Translation3d(kXandYCoord, kXandYCoord, kCameraHeight), 
            //     new Rotation3d(0, kCameraPitch, (Math.PI / 4.0))),
            //     false
            // ),
            // new CameraConstants("BackCenterCamera", new Transform3d(
            //     new Translation3d(-0.073025,0.041275,0.25146),
            //     new Rotation3d(0,0,Math.PI)),
            //     false
            // ),
            // new CameraConstants("BackRightCamera", new Transform3d(
            //     new Translation3d(0.104775,-0.352425,0.2794),
            //     new Rotation3d(0,0,Math.PI)),
            //     false
            // ),
            // new CameraConstants("BackLeftCamera", new Transform3d(
            //     new Translation3d(0.104775,0.352425, Units.inchesToMeters(13.5)),
            //     new Rotation3d(0,0,Math.PI)),
            //     false
            // )
        };
    }

    public static class ElevatorConfigs {
        public static final int kFrontElevatorID = 13; 
        public static final int kBackElevatorID = 14; 

        private static final MotionMagicConfigs kElevatorMotionMagicConfig = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(9.3)//TODO these could be faster
            .withMotionMagicAcceleration(40);

        public static final GravityTypeValue kElevatorGravityType = GravityTypeValue.Elevator_Static;

        private static final Slot0Configs kElevatorPIDConfig = new Slot0Configs()
            .withKP(4)
            .withKD(0.5)
            .withKV(1.16)
            .withKG(0.4)
            .withGravityType(kElevatorGravityType);
        //FIXME hi HARSHIL PANDENATOR

        public static final CurrentLimitsConfigs kElevatorCurrentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimit(120);

        public static final FeedbackConfigs kElevatorFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(9.0);

        public static final SoftwareLimitSwitchConfigs kElevatorSoftLimit = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(9.5) 
            .withReverseSoftLimitThreshold(0.05);

        public static final TalonFXConfiguration kElevatorConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive))
            .withMotionMagic(kElevatorMotionMagicConfig)
            .withSlot0(kElevatorPIDConfig)
            .withCurrentLimits(kElevatorCurrentLimits)
            .withFeedback(kElevatorFeedbackConfigs)
            .withSoftwareLimitSwitch(kElevatorSoftLimit);

        public static final double kElevatorTolerance = 0.015;

        //TODO change all elevator positions once we can find them
        public static final double kElevatorTrough = 3.042;
        public static final double kElevatorL1 = 1.683;
        public static final double kElevatorL2 = 3.843;
        public static final double kElevatorL3 = 8.6;
        public static final double kElevatorLowAlgae = 2.8;
        public static final double kElevatorHighAlgae = 5.3;
        public static final double kElevatorIndexerPos = 1.5;
        public static final double kElevatorProcessor = 0; 
        public static final double kElevatorSource = 4.311;
        public static final double kElevatorNet = 0;
    }

    public static final class IndexerConstants {
        public static final int kIndexerRollerID = 15;

        public static final double kIndexerRollerPower = 0.5;

        public static final CurrentLimitsConfigs kIndexerRollerCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimit(120);

        public static final TalonFXConfiguration kIndexerRollerConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withCurrentLimits(kIndexerRollerCurrentLimitConfigs);

        public static final int kPhotoelectricSensorID = 5; //TODO Change this   
    }

    public static final class IntakeConstants {
        public static final double kIntakeStartingPosition = 0; //TODO find & change

        public static final CurrentLimitsConfigs kIntakeArmMotorCurrentLimit = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimit(120);

        public static final GravityTypeValue kIntakeArmGravityType = GravityTypeValue.Arm_Cosine;

        public static final Slot0Configs kIntakeArmPidConstants = new Slot0Configs()
            .withKP(0.17)
            .withKD(0)
            .withKG(0) // TODO tune values
            .withKS(0)
            .withKV(.117)
            .withKA(0)
            .withGravityType(kIntakeArmGravityType);

        public static final SoftwareLimitSwitchConfigs kIntakeArmSoftLimit = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(60)
            .withReverseSoftLimitThreshold(0); //TODO Change

        public static final MotionMagicConfigs kIntakeArmMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(120)
            .withMotionMagicAcceleration(80); // TODO tune values

        public static final FeedbackConfigs kIntakeArmFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(50.0 / 360.0); //TODO find real value alter

        public static final TalonFXConfiguration kIntakeArmConfig = new TalonFXConfiguration()
            .withCurrentLimits(kIntakeArmMotorCurrentLimit)
            .withSlot0(kIntakeArmPidConstants)
            .withMotionMagic(kIntakeArmMotionMagicConfigs)
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(kIntakeArmFeedbackConfigs)
            .withSoftwareLimitSwitch(kIntakeArmSoftLimit);

        public static final CurrentLimitsConfigs kIntakeRollerCurrentLimit = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimit(120); 

        public static final TalonFXConfiguration kIntakeRollerConfig = new TalonFXConfiguration()
            .withCurrentLimits(kIntakeRollerCurrentLimit)
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
        
        public static final int kIntakeArmID = 16;
        public static final int kIntakeRollerID = 17;
        public static final int kIntakeDistSensorID = 30;
        public static final double kCoralDetectionDistance = 0.5; //TODO tune

        public static final double kEjectPower = -1.0;
        public static final double kEjectDelay = 0.2;
        public static final double kGroundPosition = 28828;
        public static final double kIntakePower = 1.0;
        public static final double kStowPosition = 1000;
    }

    public static class ArmConstants {
        public static final int kArmMotorId = 18;

        public static final double kArmEncoderRange = 360.0;
        public static final boolean kInvertAbsEncoder = true;
        public static final int kArmEncoderPort = 2; 
        public static final double kArmEncoderZeroPos = -16.5; 

        public static final double kArmIndexerPos = 90;
        public static final double kArmTroughPos = 200;
        public static final double kArmLevel1Pos = 242.7; //TODO change
        public static final double kArmLevel2Pos = 242.7;
        public static final double kArmLevel3Pos = 212.0;
        public static final double kArmHighAlgaePos = 190.0;
        public static final double kArmLowAlgaePos = 190.0;
        public static final double kArmProcessorPos = 0;
        public static final double kArmSourcePos = 140.0;
        public static final double kArmNetPos = 0;

        public static final double kArmPositionTolerance = 0.5;

        public static final CurrentLimitsConfigs kArmMotorCurrentLimit = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimit(120);
        
        public static final GravityTypeValue kArmGravityType = GravityTypeValue.Arm_Cosine;

        public static final Slot0Configs kArmMotorPidConstants = new Slot0Configs()
            .withKP(0.7)
            .withKD(0)
            .withKG(0)//TODO add kG
            .withKV(0.035)
            .withGravityType(kArmGravityType);

        public static final MotionMagicConfigs kArmMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(300)
            .withMotionMagicAcceleration(2000); 

        public static final FeedbackConfigs kArmFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(94.5 / 360.0);

        public static final SoftwareLimitSwitchConfigs kArmSoftLimit = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(270.0) 
            .withReverseSoftLimitThreshold(70.0);

        public static final TalonFXConfiguration kArmMotorConstants = new TalonFXConfiguration()
            .withCurrentLimits(kArmMotorCurrentLimit)
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withSlot0(kArmMotorPidConstants)
            .withMotionMagic(kArmMotionMagicConfigs)
            .withSoftwareLimitSwitch(kArmSoftLimit)
            .withFeedback(kArmFeedbackConfigs);
    }
    
    public static class ClimberConstants {
        public static final int kMotorID = 19;

        public static final int kClimberUp = 580;
        public static final int kClimberDown = -50;

        private static final Slot0Configs kClimberPIDConfig = new Slot0Configs()
            .withKP(2)
            .withKD(0)
            .withKV(0)
            .withKA(0)
            .withKS(0)
            .withKG(0);
        //FIXME hi HARSHIL PANDENATOR

        public static final CurrentLimitsConfigs kClimberCurrentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(70)
            .withStatorCurrentLimit(120);
            
        public static final FeedbackConfigs kClimberFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(125.0 / 360.0); //TODO find true value

        public static final SoftwareLimitSwitchConfigs kClimberSoftLimit = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(580)
            .withReverseSoftLimitThreshold(-50);

        public static final TalonFXConfiguration kClimberMotorConfig = new TalonFXConfiguration()
            .withSlot0(kClimberPIDConfig)
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(kClimberFeedbackConfigs)
            .withCurrentLimits(kClimberCurrentLimits)
            .withSoftwareLimitSwitch(kClimberSoftLimit);
    }

    public static final class EndEffectorConstants {
        public static final int kMainEndEffectorID = 20;

        public static final int kSecondaryEndEffectorID = 21;

        public static final int kAlgaeDistSensorID = 35;
        public static final int kSourcePhotoelectricSensorID = 0;        
        public static final int kIndexerPhotoelectricSensorID = 1;

        public static final double kProcessorOuttakePower = 0.5;
        public static final double kNetOuttakePower = 1.0;
        public static final double kIntakeAlgaePower = -1.0;
        public static final double kHoldAlgaePower = -1.0;//TODO algae is dropped due to overheating

        public static final double kCoralOuttakePower = 1.0;
        public static final double kCoralOuttakeToTrough = -0.2;
        public static final double kIntakeFromSourcePower = 0.8;
        public static final double kReceiveFromIndexerPower = -0.8;
        public static final double kSlowIntakeFromSourcePower = 0.1; 
        public static final double kSlowReceiveFromIndexerPower = -0.1; 

        public static final double kOuttakeWait = 0.5;
        public static final double kHasAlgaeDist = 182;

        public static final CurrentLimitsConfigs kCurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(120)
            .withSupplyCurrentLimit(40);

        public static final MotorOutputConfigs kMainEndEffectorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake); 

        public static final MotorOutputConfigs kSecondaryEndEffectorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake); 

        public static final TalonFXConfiguration kMainEndEffectorConfig = new TalonFXConfiguration()
            .withCurrentLimits(kCurrentLimits)
            .withMotorOutput(kMainEndEffectorOutputConfigs);

        public static final TalonFXConfiguration kSecondaryEndEffectorConfig = new TalonFXConfiguration()
            .withCurrentLimits(kCurrentLimits)
            .withMotorOutput(kSecondaryEndEffectorOutputConfigs);
    }

    public static final class ButtonBoardConstants {
        public static final int kButtonBoardPort1 = 3;
        public static final int kButtonBoardPort2 = 4;
    }
}