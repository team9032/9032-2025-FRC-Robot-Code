package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.SwerveConstants;

public final class Constants {
    public static class DriverConstants {
        public static final boolean kRunSysId = false;

        public static final String kCANBusName = "canivore";

        public static final double kLowStartingBatteryVoltage = 12.2;

        public static final int kDriveControllerPort = 0;
        public static final double kRumbleTime = 0.3;

        public static final double kIntakeDriverAssistStartTime = 0.25;//Seconds 
        public static final double kHasCoralDebounceTime = 0.1;//Seconds 

        public static final double kMaxSpeed = SwerveConstants.kSpeedAt12Volts.magnitude();
        public static final double kRotationRate = 4 * Math.PI;

        public static final double kJoystickDeadband = 0.01;//Percent of velocity

        public final static FieldCentric kDriveRequest = new FieldCentric()
            .withDeadband(kMaxSpeed * kJoystickDeadband) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

        public static final boolean kUseAutoIntake = true;
    }

    public static class PathFollowingConstants {
        /* Barge alignment constants */
        public static final double kBargeAlignmentX = 7.6 + Units.inchesToMeters(8);
        public static final Rotation2d kBargeAlignmentRotation = Rotation2d.kZero;
        public static final double kBargeMaxY = 4.95;

        /* Scoring and intaking offsets */
        public static final Transform2d kLeftScoringOffset = new Transform2d(0.585, -0.125, Rotation2d.kZero);
        public static final Transform2d kRightScoringOffset = new Transform2d(0.585, 0.225, Rotation2d.kZero);
        public static final Transform2d kBackwardsScoringOffset = new Transform2d(Units.inchesToMeters(6.0), 0, Rotation2d.kZero);
        public static final Transform2d kAlgaeReefIntakeOffset = new Transform2d(0.64, 0.0, Rotation2d.kZero);

        /* Reef distances */
        public static final double kPrepareForScoringReefDistance = 1.8;
        public static final double kPrepareForAlgaeIntakingReefDistance = 2.04;
        public static final double kEndEffectorClearReefDistance = 1.55;
        public static final double kEndEffectorClearReefDistanceWithAlgae = 1.7;

        public static final double kPrepareForNetAlgaeScoringDistance = Units.inchesToMeters(24.0);

        /* Drive to coral */
        public static final double kMaxDrivingSpeed = 3.0;
        public static final double kSlowDrivingSpeed = 1.0;
        public static final double kSlowDistanceToCoral = 1.3;
        public static final double kEndTime = 0.3;//Seconds

        /* Pull Away */
        public static final double kPullAwayVelocity = 1.0;

        public static final double kAlgaeIntakeWait = 0.075;
    }
    
    public static final class IntakeDriverAssistConstants {
        public static final String kObjectTrackingCameraName = "FrontCenterCamera";

        /* Rotational */
        public static final double kPRotationToObject = 0.15;
        public static final double kDRotationToObject = 0.002;
        public static final double kRotationSetpoint = 10.65;
        public static final double kEndDistanceToCoral = 0.95;

        /* Translational */
        public static final double kPTranslation = 0.25;
    }

    public static final class ButtonBoardConstants {
        public static final int kButtonBoardPort1 = 3;
        public static final int kButtonBoardPort2 = 4;
        public static final int kButtonBoardPort3 = 5;
    }
}