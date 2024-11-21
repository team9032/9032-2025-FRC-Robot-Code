package frc.robot.subsystems.swerve;

import static frc.robot.Constants.PathplannerConfig.kRobotConfig;
import static frc.robot.Constants.PathplannerConfig.kRotationPID;
import static frc.robot.Constants.PathplannerConfig.kTranslationPID;
import static frc.robot.subsystems.swerve.SwerveConstants.backLeft;
import static frc.robot.subsystems.swerve.SwerveConstants.backRight;
import static frc.robot.subsystems.swerve.SwerveConstants.drivetrainConstants;
import static frc.robot.subsystems.swerve.SwerveConstants.frontLeft;
import static frc.robot.subsystems.swerve.SwerveConstants.frontRight;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KrakenSwerve extends SubsystemBase {
    public final SwerveDrivetrain driveTrain;

    public KrakenSwerve() {
        driveTrain = new SwerveDrivetrain(drivetrainConstants, frontLeft, frontRight, backLeft, backRight);

        /* Configure PathPlanner */
        AutoBuilder.configure(
            () -> driveTrain.getState().Pose, 
            driveTrain::resetPose, 
            () -> driveTrain.getState().Speeds, 
            this::driveClosedLoop, 
            new PPHolonomicDriveController(
                kTranslationPID, 
                kRotationPID
            ), 
            kRobotConfig, 
            /* Supplier for if the path should be mirrored for the red alliance - will always use blue for origin */
            () -> DriverStation.getAlliance().orElseGet(() -> DriverStation.Alliance.Blue) == DriverStation.Alliance.Red, 
            this
        );
    }

    public Command zeroGyro() {
        return runOnce(() -> driveTrain.getPigeon2().reset());
    }

    private void driveClosedLoop(ChassisSpeeds setpoint, DriveFeedforwards feedforwards) {
        //TODO implement
    }
}
