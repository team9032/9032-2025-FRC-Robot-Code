package frc.robot;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import org.photonvision.estimation.TargetModel;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
//import harshil.pande.TigerConstants;
//import evilharshel.pandez.EvilTigerConstantz.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.localization.CameraConstants;
import frc.robot.subsystems.swerve.SwerveConstants;

public final class Constants {
    public static class DriverConstants {
        public static final boolean kRunSysId = false;

        public static final String kCANBusName = "canivore";

        public static final double kLowStartingBatteryVoltage = 12.2;

        public static final int kDriveControllerPort = 0;

        public static final double kOverrideAutomationThreshold = 0.1;
        public static final double kIntakeDriverAssistStartTime = 0.25;//Seconds 

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

    public static class PathFollowingConstants {
        /* Pathplanner constants */
        public static final PIDConstants kTranslationPID = new PIDConstants(10.0);
        public static final PIDConstants kRotationPID = new PIDConstants(7.0);

        public static final ApplyRobotSpeeds kRobotRelativeClosedLoopDriveRequest = new ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

        public static final PathConstraints kDynamicPathConstraints = new PathConstraints(
            3.7,
            3.5, 
            3 * Math.PI, 
            4 * Math.PI
        );

        public static final FieldCentric kFieldCentricClosedLoopDriveRequest = new FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

        /* Drive to pose constants */
        public static final double kAlignmentXYkP = 10.0;
        public static final double kAlignmentXYkD = 0.1;
        
        public static final double kAlignmentRotkP = 8.0;
        public static final double kAlignmentRotkD = 0.1;

        public static final double kXYAlignmentTolerance = Units.inchesToMeters(0.33);
        public static final double kRotAlignmentTolerance = Units.degreesToRadians(4);

        /* Barge alignment constants */
        public static final double kBargeAlignmentX = 7.6;
        public static final Rotation2d kBargeAlignmentRotation = Rotation2d.kZero;

        /* Intake offsets */
        public static final Transform2d kCoralIntakeOffset = new Transform2d(-0.6, 0, Rotation2d.kZero);//TODO find
        public static final Transform2d kAlgaeReefIntakeOffset = new Transform2d(0.64, 0.0, Rotation2d.kZero);

        /* Scoring offsets */
        public static final Transform2d kLeftScoringOffset = new Transform2d(0.585, -0.125, Rotation2d.kZero);
        public static final Transform2d kRightScoringOffset = new Transform2d(0.585, 0.225, Rotation2d.kZero);

        /* Reef distances */
        public static final double kPrepareForScoringReefDistance = 1.8;
        public static final double kPrepareForAlgaeIntakingReefDistance = 2.04;
        public static final double kEndEffectorClearReefDistance = 1.55;
        public static final double kEndEffectorClearReefDistanceWithAlgae = 1.7;

        public static final double kPrepareForNetAlgaeScoringDistance = Units.inchesToMeters(24.0);

        /* Drive to coral */
        public static final double kMaxDrivingSpeed = 3.0;
        public static final double kSlowDrivingSpeed = 1.0;
        public static final double kSlowDistanceToCoral = Units.feetToMeters(4);//TODO Find
    }
    
    public static final class IntakeDriverAssistConstants {
        public static final String kObjectTrackingCameraName = "FrontCenterCamera";

        /* Rotational */
        public static final double kPRotationToObject = 0.15;
        public static final double kDRotationToObject = 0.002;
        public static final double kRotationSetpoint = 13.3;
        public static final double kEndDistanceToCoral = Units.feetToMeters(4);//TODO Find

        /* Translational */
        public static final double kPTranslation = 0.25;
    }

    public static final class LocalizationConstants {
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

        /* Class Ids */
        public static final int kCoralId = 1;
        public static final int kAlgaeId = 0;

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
            // new CameraConstants("BackRightCamera", new Transform3d(
            //     new Translation3d(Units.inchesToMeters(-0.25),Units.inchesToMeters(-13.5625),Units.inchesToMeters(7.875)),
            //     new Rotation3d(0,Units.degreesToRadians(-20),Math.PI)),
            //     false
            // ),
            // new CameraConstants("GroundCamera", new Transform3d(
            //     new Translation3d(Units.inchesToMeters(-14.375),Units.inchesToMeters(5.875), Units.inchesToMeters(30.875)),
            //     new Rotation3d(0,Units.degreesToRadians(-20), Math.PI)),
            //     true//TODO find offsets for this camera
            // )
        };

        /* Field constants from the game manual */
        public static final Translation2d kReefCenter = new Translation2d(Units.inchesToMeters(176.746), 8.052 / 2.0);
        public static final TargetModel kCoralModel = new TargetModel(Units.inchesToMeters(11.875), Units.inchesToMeters(4.5), Units.inchesToMeters(4.5));
        public static final int kMinReefTagID = 17;
        public static final int kMaxReefTagID = 22;
        public static final int kBackReefTagsStartingID = 20;
    }

    public static class ElevatorConfigs {
        public static final int kFrontElevatorID = 13; 
        public static final int kBackElevatorID = 14; 

        private static final MotionMagicConfigs kElevatorMotionMagicConfig = new MotionMagicConfigs()
            .withMotionMagicExpo_kV(0.2)
            .withMotionMagicExpo_kA(0.16);

        public static final GravityTypeValue kElevatorGravityType = GravityTypeValue.Elevator_Static;

        private static final Slot0Configs kElevatorPIDConfig = new Slot0Configs()
            .withKP(13)
            .withKD(1)
            .withKV(0.6)
            .withKG(0.4)
            .withGravityType(kElevatorGravityType);

        public static final CurrentLimitsConfigs kElevatorCurrentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(60)
            .withStatorCurrentLimit(120);

        public static final FeedbackConfigs kElevatorFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(5.0);

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

        public static final double kElevatorL1 = 3.1;
        public static final double kElevatorL2 = 1.3;
        public static final double kElevatorL3 = 3.7;
        public static final double kElevatorL4 = 6.7;
        public static final double kElevatorLowAlgae = 3.5;
        public static final double kElevatorHighAlgae = 5.7;
        public static final double kElevatorCradlePos = 2.3;
        public static final double kElevatorProcessor = 0; 
        public static final double kElevatorNet = 9.1;
        public static final double kElevatorAlgaeGround = 2.0;
        public static final double kElevatorClimb = 0.05;

        public static final double kElevatorOverCradle = 3;
        public static final double kElevatorOverHighAlgae = 4.5;
        public static final double kElevatorStow = 1.6; 
        public static final double kElevatorCloseToNet = 8.8;
    }

    public static final class TransferConstants {
        public static final int kTransferRollerID = 15;
        public static final int kTransferPhotoelectricSensorID = 0;

        public static final double kTransferRollerPower = -1.0;
        public static final double kTransferRollerSlowPower = -0.2;

        public static final double kTransferFromSensorWait = 0.25;

        public static final double kTransferEjectPower = 0.25;
        public static final double kTransferEjectWait = 1.0;

        public static final CurrentLimitsConfigs kTransferRollerCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimit(120);

        public static final TalonFXConfiguration kTransferRollerConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withCurrentLimits(kTransferRollerCurrentLimitConfigs);
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
            .withReverseSoftLimitThreshold(-120.0); 

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

        public static final double kGroundPosition = -130.0;
        public static final double kStowPosition = -27.0;
        public static final double kEndEffectorMovePosition = -94.0;

        public static final double kRunRollersPosition = -104;

        public static final double kCanClimbPosition = -34.0;
    }

    public static class ArmConstants {
        public static final int kArmMotorId = 18;

        public static final int kArmEncoderId = 38;
        public static final MagnetSensorConfigs kArmEncoderConfig = new MagnetSensorConfigs()
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(0.5)
            .withMagnetOffset(-0.1032);

        public static final double kArmStowPos = 0.2;
        public static final double kArmCradlePos = -0.25;
        public static final double kArmL2ScorePos = 0.03; 
        public static final double kArmL3ScorePos = 0.01;
        public static final double kArmL4ScorePos = 0.05;
        public static final double kArmHighAlgaePos = 0.0;
        public static final double kArmLowAlgaePos = 0.0;
        public static final double kArmProcessorPos = 0;
        public static final double kArmNetPos = 0.38;
        public static final double kArmAlgaeGroundPos = -0.1;
        public static final double kArmClimbPos = 0.09;
        public static final double kArmCoralPreparedToScorePos = 0.17;
        public static final double kArmLowL1PreparedToScorePos = -0.06;
        public static final double kArmHighL1PreparedToScorePos = -0.06;

        public static final double kArmOverCradlePos = 0.0;
        public static final double kArmPositionTolerance = 0.005;

        public static final CurrentLimitsConfigs kArmMotorCurrentLimit = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimit(120);
        
        public static final GravityTypeValue kArmGravityType = GravityTypeValue.Arm_Cosine;

        public static final Slot0Configs kArmMotorPidConstants = new Slot0Configs()
            .withKG(0.3)
            .withKP(130)
            .withKD(0.1)
            .withKA(0.25)
            .withKV(4.5)
            .withGravityType(kArmGravityType);

        public static final MotionMagicConfigs kArmMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicExpo_kV(2.5)
            .withMotionMagicExpo_kA(1.5); 

        public static final FeedbackConfigs kArmFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(1.0)
            .withRotorToSensorRatio(162240.0 / 3456.0)
            .withFeedbackRemoteSensorID(kArmEncoderId)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder);

        public static final TalonFXConfiguration kArmMotorConstants = new TalonFXConfiguration()
            .withCurrentLimits(kArmMotorCurrentLimit)
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withSlot0(kArmMotorPidConstants)
            .withMotionMagic(kArmMotionMagicConfigs)
            .withFeedback(kArmFeedbackConfigs);
    }
    
    public static class ClimberConstants {
        public static final int kClimberArmID = 32;
        public static final int kClimberIntakeID = 33;

        public static final double kClimberStowPos = 0.11;
        public static final double kClimberCageIntakePos = 0.2;
        public static final double kClimberClimbPos = -0.08;

        private static final Slot0Configs kClimberPIDConfig = new Slot0Configs()
            .withKP(10000)
            .withKD(0)
            .withKV(0)
            .withKA(0)
            .withKS(0)
            .withKG(0);
        //FIXME hi HARSHIL PANDENATOR

        public static final CurrentLimitsConfigs kClimberArmCurrentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(70)
            .withStatorCurrentLimit(120);
            
        public static final FeedbackConfigs kClimberArmFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(432.0); 

        public static final SoftwareLimitSwitchConfigs kClimberSoftLimit = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(0.25)
            .withReverseSoftLimitThreshold(-0.08);

        public static final TalonFXConfiguration kClimberArmMotorConfig = new TalonFXConfiguration()
            .withSlot0(kClimberPIDConfig)
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(kClimberArmFeedbackConfigs)
            .withCurrentLimits(kClimberArmCurrentLimits)
            .withSoftwareLimitSwitch(kClimberSoftLimit);

        public static final TalonFXConfiguration kClimberIntakeMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(120)
                .withSupplyCurrentLimit(40)
            );

        public static final double kClimberIntakeVolts = 6.0;
        public static final int kHasCageCurrent = 94;
        public static final double kCageIntakeDelay = 0.5;

        public static final double kClimberArmTolerance = 0.005;
    }

    public static final class EndEffectorConstants {
        public static final int kEndEffectorRollerMotorID = 20;

        public static final double kProcessorOuttakePower = 1.0;
        public static final double kNetOuttakePower = 1.0;

        public static final double kCoralOuttakePower = 0.02;
        public static final double kCoralOuttakeToL4Power = 0.5;
        public static final double kCoralOuttakeToTroughPower = 0.5;

        public static final double kCoralOuttakeWait = 0.05;
        public static final double kCoralOuttakeWaitToTrough = 0.5;
        public static final double kCoralOuttakeWaitToL4 = 0.2;
        public static final double kAlgaeOuttakeWait = 0.5;

        public static final double kHoldAlgaeCurrent = -60;
        public static final double kHoldCoralCurrent = -15;

        public static final int kHasCoralCurrent = -14;
        public static final int kHasAlgaeCurrent = -57;

        public static final CurrentLimitsConfigs kEndEffectorCurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(120)
            .withSupplyCurrentLimit(40);

        public static final MotorOutputConfigs kEndEffectorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake); 

        public static final TalonFXConfiguration kEndEffectorRollerMotorConfig = new TalonFXConfiguration()
            .withCurrentLimits(kEndEffectorCurrentLimits)
            .withMotorOutput(kEndEffectorOutputConfigs);
    }

    public static final class ButtonBoardConstants {
        public static final int kButtonBoardPort1 = 3;
        public static final int kButtonBoardPort2 = 4;
        public static final int kButtonBoardPort3 = 5;
    }

    public static final class LEDConstants {
        public static final int kLEDPort = 0;
        public static final int kLEDLength = 99;
        public static final Distance kLedSpacing = Meters.of(1.0 / 120.0);

        //Patterns
        public static final LEDPattern kError = LEDPattern.solid(Color.kDarkRed);

        public static final LEDPattern kClimbing = LEDPattern.rainbow(255, 255);

        public static final LEDPattern kBootingUp = LEDPattern.solid(Color.kPurple);

        public static final LEDPattern kDisabledPattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kDarkBlue, Color.kBlue, Color.kAquamarine)
            .scrollAtAbsoluteSpeed(InchesPerSecond.of(8.0), kLedSpacing)
            .breathe(Seconds.of(2));

        public static final LEDPattern kEnabledPattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kLimeGreen, Color.kDarkGreen)
            .scrollAtAbsoluteSpeed(InchesPerSecond.of(20.0), kLedSpacing)
            .synchronizedBlink(() -> RobotController.getRSLState());

        public static final LEDPattern kL1Pattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kWhite, Color.kBlack)
            .scrollAtAbsoluteSpeed(InchesPerSecond.of(100.0), kLedSpacing);

        public static final LEDPattern kL2Pattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kYellow, Color.kLightGoldenrodYellow)
            .scrollAtAbsoluteSpeed(InchesPerSecond.of(100.0), kLedSpacing);

        public static final LEDPattern kL3Pattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kDarkOliveGreen, Color.kDarkGreen)
            .scrollAtAbsoluteSpeed(InchesPerSecond.of(100.0), kLedSpacing);

        public static final LEDPattern kL4Pattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kDarkBlue, Color.kBlueViolet)
            .scrollAtAbsoluteSpeed(InchesPerSecond.of(100.0), kLedSpacing);

        public static final LEDPattern kAlgaePattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous,  Color.kDarkBlue, Color.kPurple, Color.kDarkViolet)
            .scrollAtAbsoluteSpeed(InchesPerSecond.of(100.0), kLedSpacing);

        public static final LEDPattern kBatteryLowPattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kDarkRed, Color.kOrange)
            .scrollAtAbsoluteSpeed(InchesPerSecond.of(8.0), kLedSpacing)
            .blink(Seconds.of(0.6));
    }
}