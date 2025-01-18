package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.localization.Localization;

import static frc.robot.Constants.PathplannerConfig.*;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

public class KrakenSwerve extends SubsystemBase {
    public final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;
    
    private final Localization localization;

    public KrakenSwerve() {
        drivetrain = new SwerveDrivetrain<>(
            TalonFX::new, TalonFX::new, CANcoder::new,
            drivetrainConstants, kFrontLeft, kFrontRight, kBackLeft, kBackRight
        );

        /* Configure PathPlanner */
        AutoBuilder.configure(
            () -> drivetrain.getState().Pose, 
            drivetrain::resetPose, 
            () -> drivetrain.getState().Speeds, 
            this::drivePathPlanner, 
            new PPHolonomicDriveController(
                kTranslationPID, 
                kRotationPID
            ), 
            kRobotConfig, 
            /* Supplier for if the path should be mirrored for the red alliance - will always use blue for origin */
            () -> DriverStation.getAlliance().orElseGet(() -> DriverStation.Alliance.Blue) == DriverStation.Alliance.Red, 
            this
        );
        
        localization = new Localization(drivetrain);
        localization.switchAllToLocalization();
    }

    public Command zeroGyro() {
        return runOnce(() -> drivetrain.getPigeon2().reset());
    }

    private void drivePathPlanner(ChassisSpeeds setpoint, DriveFeedforwards feedforwards) {
        drivetrain.setControl(
            kPathPlannerDriveRequest.withSpeeds(setpoint)
            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX())
            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY())
        );
    }

    @Override
    public void periodic() {
        localization.updateLocalization();
    }
}