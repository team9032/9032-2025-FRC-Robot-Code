package frc.robot.subsystems.swerve;

import static frc.robot.Constants.PathplannerConfig.kMaxSteerVelocity;
import static frc.robot.Constants.PathplannerConfig.kModuleRequest;
import static frc.robot.Constants.PathplannerConfig.kPathPlannerDriveRequest;
import static frc.robot.Constants.PathplannerConfig.kRobotConfig;
import static frc.robot.Constants.PathplannerConfig.kRotationPID;
import static frc.robot.Constants.PathplannerConfig.kTranslationPID;
import static frc.robot.subsystems.swerve.SwerveConstants.backLeft;
import static frc.robot.subsystems.swerve.SwerveConstants.backRight;
import static frc.robot.subsystems.swerve.SwerveConstants.drivetrainConstants;
import static frc.robot.subsystems.swerve.SwerveConstants.frontLeft;
import static frc.robot.subsystems.swerve.SwerveConstants.frontRight;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KrakenSwerve extends SubsystemBase {
    private final SwerveDrivetrain driveTrain;

    private final SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;

    public KrakenSwerve() {
        driveTrain = new SwerveDrivetrain(drivetrainConstants, frontLeft, frontRight, backLeft, backRight);

        /* Configure PathPlanner */
        AutoBuilder.configure(
            () -> driveTrain.getState().Pose, 
            driveTrain::resetPose, 
            () -> driveTrain.getState().Speeds, 
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

        setpointGenerator = new SwerveSetpointGenerator(kRobotConfig, kMaxSteerVelocity);
        previousSetpoint = new SwerveSetpoint(
            driveTrain.getState().Speeds, 
            driveTrain.getState().ModuleStates, 
            DriveFeedforwards.zeros(4)
        );
    }

    public Command zeroGyro() {
        return runOnce(() -> driveTrain.getPigeon2().reset());
    }

    public void driveOptimized(ChassisSpeeds setpoint, DriveRequestType driveRequestType) {//TODO should this method be used with PathPlanner?
        SwerveSetpoint optimizedSetpoint = setpointGenerator.generateSetpoint(previousSetpoint, setpoint, 0.02);
        previousSetpoint = optimizedSetpoint;
    
        for(int i = 0; i < 4; i++) {
            SwerveModule module = driveTrain.getModule(i);
            
            ModuleRequest request = kModuleRequest
                .withDriveRequest(driveRequestType)
                .withState(optimizedSetpoint.moduleStates()[i])
                .withWheelForceFeedforwardX(optimizedSetpoint.feedforwards().robotRelativeForcesX()[i])
                .withWheelForceFeedforwardY(optimizedSetpoint.feedforwards().robotRelativeForcesY()[i]);

            module.apply(request);
        }
    }

    private void drivePathPlanner(ChassisSpeeds setpoint, DriveFeedforwards feedforwards) {
        driveTrain.setControl(
            kPathPlannerDriveRequest.withSpeeds(setpoint)
            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX())
            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY())
        );
    }
}
