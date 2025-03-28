package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.localization.CameraConstants;
import frc.robot.subsystems.swerve.SwerveConstants;

public final class Constants {
    public static class DriverConstants {
        public static final boolean kRunSysId = false;

        public static final double kLowStartingBatteryVoltage = 12.2;

        public static final int kDriveControllerPort = 0;

        public static final double kOverrideAutomationThreshold = 0.1;

        public static final double kMaxSpeed = SwerveConstants.kSpeedAt12Volts.magnitude();
        public static final double kSlowSpeed = kMaxSpeed * 0.15;
        public static final double kRotationRate = 4 * Math.PI;
        public static final double kSlowRotationRate = kRotationRate * 0.15;

        public static final double kRumbleTime = 0.5;

        public final static FieldCentric kDriveRequest = new FieldCentric()
            .withDeadband(kMaxSpeed * 0.01) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    }

    public static class PathplannerConfig {
        public static final PIDConstants kTranslationPID = new PIDConstants(10.0);
        public static final PIDConstants kRotationPID = new PIDConstants(7.0);

        public static final ApplyRobotSpeeds kClosedLoopDriveRequest = new ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

        public static final PathConstraints kDynamicPathConstraints = new PathConstraints(
            3.0,//TODO change these
            6, 
            2 * Math.PI, 
            2 * Math.PI
        );

        public static final double kAlignmentXYkP = 2;//TODO Tune better
        public static final double kAlignmentXYkD = 0;
        
        public static final double kAlignmentRotkP = 10.0;
        public static final double kAlignmentRotkD = 0;

        public static final double kXYAlignmentTolerance = Units.inchesToMeters(0.25);
        public static final double kRotAlignmentTolerance = Units.degreesToRadians(2);
    }

    public static final class ObjectAimingConstants {
        public static final String kObjectTrackingCameraName = "FrontCenterCamera";

        public static final int kCycleAmtSinceTargetSeenCutoff = 10;
        public static final double kPitchDifferenceCutoff = 2.0;

        public static final double kRotationSetpoint = 13.3;
        public static final double kMaxDrivingSpeed = 1.0;//Meters per second

        public static final double kSlowObstacleDistance = 1.20;//Meters from sensor
        public static final double kSlowDrivingSpeed = 0.25;

        /* PID Constants */
        public static final double kPRotation = 0.15;
        public static final double kDRotation = 0.002;

        /* Class Ids */
        public static final int kCoralId = 1;
        public static final int kAlgaeId = 0;
    }

    public static final class LocalizationConstants {
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

        public static final CameraConstants[] kCameraConstants = new CameraConstants[] {
            new CameraConstants("FrontCenterCamera", new Transform3d(
                new Translation3d(0, 0, 0), //TODO find offsets for this camera
                new Rotation3d(0, 0,  -(Math.PI / 4.0))),
                true
            ),
            // new CameraConstants("FrontRightCamera", new Transform3d(
            //     new Translation3d(kXandYCoord, -kXandYCoord, kCameraHeight), 
            //     new Rotation3d(0, kCameraPitch,  -(Math.PI / 4.0))),
            //    false
            // ),
            new CameraConstants("BackLeftCamera", new Transform3d(
               new Translation3d(Units.inchesToMeters(3.875), Units.inchesToMeters(14.5), Units.inchesToMeters(8.375)), 
               new Rotation3d(0,Units.degreesToRadians(-20), Units.degreesToRadians(146))),
               false
            ),
            new CameraConstants("BackCenterCamera", new Transform3d(
               new Translation3d(Units.inchesToMeters(-14.375),Units.inchesToMeters(6.0),Units.inchesToMeters(6.75)),
               new Rotation3d(0,Units.degreesToRadians(-20),Math.PI)),
               false
            ),
            new CameraConstants("BackRightCamera", new Transform3d(
                new Translation3d(Units.inchesToMeters(-0.25),Units.inchesToMeters(-13.5625),Units.inchesToMeters(7.875)),
                new Rotation3d(0,Units.degreesToRadians(-20),Math.PI)),
                false
            ),
            new CameraConstants("FrontLeftCamera", new Transform3d(
                new Translation3d(Units.inchesToMeters(-14.375),Units.inchesToMeters(5.875), Units.inchesToMeters(30.875)),
                new Rotation3d(0,Units.degreesToRadians(-20),Math.PI)),
                false
            )
        };
    }

    public static class ElevatorConfigs {
        public static final int kFrontElevatorID = 13; 
        public static final int kBackElevatorID = 14; 

        private static final MotionMagicConfigs kElevatorMotionMagicConfig = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(9.5)
            .withMotionMagicAcceleration(40);

        public static final GravityTypeValue kElevatorGravityType = GravityTypeValue.Elevator_Static;

        private static final Slot0Configs kElevatorPIDConfig = new Slot0Configs()
            .withKP(7)
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

        public static final double kElevatorTolerance = 0.05;

        public static final double kElevatorTrough = 0.4;
        public static final double kElevatorL1 = 1.1;
        public static final double kElevatorL2 = 3.42;
        public static final double kElevatorL3 = 8.70;
        public static final double kElevatorLowAlgae = 4.8;
        public static final double kElevatorHighAlgae = 6.6;
        public static final double kElevatorIndexerPos = 1.6;
        public static final double kElevatorProcessor = 0; 
        public static final double kElevatorSource = 4.311;
        public static final double kElevatorNet = 9.5;

        public static final double kElevatorOverIndexer = 3.2;
        public static final double kElevatorStow = 1.6; 
    }

    public static final class IndexerConstants {
        public static final int kIndexerRollerID = 15;

        public static final double kIndexerRollerPower = -1.0;

        public static final double kIndexerEjectPower = 0.25;
        public static final double kIndexerEjectWait = 1.0;

        public static final CurrentLimitsConfigs kIndexerRollerCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimit(120);

        public static final TalonFXConfiguration kIndexerRollerConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withCurrentLimits(kIndexerRollerCurrentLimitConfigs);

        public static final int kPhotoelectricSensorID = 5; //TODO doesn't exist   
    }

    public static final class IntakeConstants {
        public static final CurrentLimitsConfigs kIntakeArmMotorCurrentLimit = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimit(120);

        public static final GravityTypeValue kIntakeArmGravityType = GravityTypeValue.Arm_Cosine;

        public static final Slot0Configs kIntakeArmPidConstants = new Slot0Configs()
            .withKP(0.5)
            .withKV(0.023)
            .withGravityType(kIntakeArmGravityType);

        public static final SoftwareLimitSwitchConfigs kIntakeArmSoftLimit = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(0.0)
            .withReverseSoftLimitThreshold(-95.0); 

        public static final MotionMagicConfigs kIntakeArmMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(300)
            .withMotionMagicAcceleration(1000); 

        public static final FeedbackConfigs kIntakeArmFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(50.0 / 360.0); 

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

        public static final double kEjectPower = 0.25;
        public static final double kEjectDelay = 0.75;

        public static final double kIntakePower = -1.0;

        public static final double kGroundPosition = -120.0;
        public static final double kStowPosition = -23.0;
        public static final double kEndEffectorMovePosition = -49.0;

        public static final double kRunRollersPosition = -100;

        public static final int kObstacleSensorID = 36;
        public static final double kDefaultObstacleDistance = 10.0;
    }

    public static class ArmConstants {
        public static final int kArmMotorId = 18;

        public static final double kArmEncoderRange = 1.0;
        public static final boolean kInvertAbsEncoder = true;
        public static final int kArmEncoderPort = 2; 
        public static final double kArmEncoderZeroPos = 0.447; 

        public static final double kArmStowPos = 0.2;
        public static final double kArmIndexerPos = 0.75;
        public static final double kArmTroughPos = 0.14;
        public static final double kArmLevel1Pos = kArmStowPos; 
        public static final double kArmLevel2Pos = kArmStowPos;
        public static final double kArmLevel3Pos = 0.045;
        public static final double kArmHighAlgaePos = -0.05;
        public static final double kArmLowAlgaePos = -0.05;
        public static final double kArmProcessorPos = 0;
        public static final double kArmSourcePos = 0.0;
        public static final double kArmNetPos = 0.1;

        public static final double kArmOverIntakePos = 0.5;
        public static final double kArmPositionTolerance = 0.005;

        public static final CurrentLimitsConfigs kArmMotorCurrentLimit = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimit(120);
        
        public static final GravityTypeValue kArmGravityType = GravityTypeValue.Arm_Cosine;

        public static final Slot0Configs kArmMotorPidConstants = new Slot0Configs()
            .withKG(0.28)
            .withKP(130.0)
            .withKD(10.0)
            .withKV(11.5)
            .withGravityType(kArmGravityType);

        public static final MotionMagicConfigs kArmMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(0.9)
            .withMotionMagicAcceleration(5.0); 

        public static final FeedbackConfigs kArmFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(94.5);

        public static final SoftwareLimitSwitchConfigs kArmSoftLimit = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(0.86) 
            .withReverseSoftLimitThreshold(-0.12);

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

        public static final double kProcessorOuttakePower = -1.0;
        public static final double kNetOuttakePower = -1.0;
        public static final double kIntakeAlgaePower = 1.0;
        public static final double kHoldAlgaePower = 0.05;

        public static final double kCoralOuttakePower = 1.0;
        public static final double kCoralOuttakeToTrough = -0.2;
        public static final double kIntakeFromSourcePower = 0.8;
        public static final double kReceiveFromIndexerPower = -1.0;
        public static final double kSlowIntakeFromSourcePower = 0.4; 
        public static final double kSlowReceiveFromIndexerPower = -0.5; 

        public static final double kOuttakeWait = 0.5;
        public static final double kHasAlgaeDist = 182;

        public static final CurrentLimitsConfigs kCurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(120)
            .withSupplyCurrentLimit(40);

        public static final MotorOutputConfigs kMainEndEffectorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
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

    public static final class AutomationConstants {
        public static final int kButtonBoardPort1 = 3;
        public static final int kButtonBoardPort2 = 4;
        public static final int kButtonBoardPort3 = 5;

        public static final Rectangle2d kIntakeZoneRectangle = new Rectangle2d(
            new Pose2d(2.04, 6.04, Rotation2d.fromDegrees(-55.0)), 0.8, 4.0);
    }

    public static final class LEDConstants {
        public static final int kLEDPort = 0;
        public static final int kLEDLength = 160;
        public static final Distance kLedSpacing = Meters.of(1/120.0);

        //Patterns
        public static final LEDPattern kBootingUp = LEDPattern.solid(Color.kRed);

        public static final LEDPattern kBaseDisabled = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kOrange);
        public static final LEDPattern kDisabledPattern = kBaseDisabled.breathe(Seconds.of(2));

        public static final LEDPattern kBaseEnabled = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kLimeGreen, Color.kDarkGreen);
        public static final LEDPattern kEnabledPattern = kBaseEnabled.blink(Seconds.of(0.6));

        public static final LEDPattern kBaseL1 = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kWhite, Color.kWhiteSmoke);
        public static final LEDPattern kL1Mask = LEDPattern.progressMaskLayer(() -> 0.35);
        public static final LEDPattern kL1Pattern = kBaseL1.mask(kL1Mask);

        public static final LEDPattern kBaseL2 = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kYellow, Color.kLightGoldenrodYellow);
        public static final LEDPattern kL2Mask = LEDPattern.progressMaskLayer(() -> 0.60);
        public static final LEDPattern kL2Pattern = kBaseL2.mask(kL2Mask);

        public static final LEDPattern kBaseL3 = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kGreen, Color.kLightSeaGreen);
        public static final LEDPattern kL3Mask = LEDPattern.progressMaskLayer(() -> 0.75);
        public static final LEDPattern kL3Pattern = kBaseL3.mask(kL3Mask);

        public static final LEDPattern kBaseL4 = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kDarkBlue, Color.kSteelBlue);
        public static final LEDPattern kL4Mask = LEDPattern.progressMaskLayer(() -> 1.00);
        public static final LEDPattern kL4Pattern = kBaseL4.mask(kL4Mask);

        public static final LEDPattern kAlgaeBase = LEDPattern.gradient(LEDPattern.GradientType.kContinuous,  Color.kAqua, Color.kLightGreen, Color.kLightCyan);
        public static final LEDPattern kAlgaePattern = kAlgaeBase.scrollAtRelativeSpeed(Percent.per(Second).of(25));

        public static final LEDPattern kBatteryLowBase = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kDarkRed, Color.kRed);
        public static final LEDPattern kBatteryLowPattern = kBaseEnabled.blink(Seconds.of(0.6));
    }
}