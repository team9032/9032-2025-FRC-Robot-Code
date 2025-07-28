package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.localization.Localization;
import frc.robot.simulation.MapleSimSwerveDrivetrain;
import frc.robot.utils.ElasticUtil;

import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Inches;
import static frc.robot.Constants.PathFollowingConstants.*;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

public class KrakenSwerve extends SubsystemBase {
    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;
    
    private final SwerveSysId sysId;

    private final Localization localization;

    private boolean operatorPerspectiveSet = false;

    private MapleSimSwerveDrivetrain simulatedDrivetrain;

    private RobotConfig pathplannerConfig;

    public KrakenSwerve() {
        drivetrain = new SwerveDrivetrain<>(
            TalonFX::new, TalonFX::new, CANcoder::new,
            kDrivetrainConstants, 
            MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(kFrontLeft, kFrontRight, kBackLeft, kBackRight)
        );

        if (RobotBase.isSimulation()) {
            simulatedDrivetrain = new MapleSimSwerveDrivetrain(
                Pounds.of(115),
                Inches.of(30),
                Inches.of(30),
                DCMotor.getKrakenX60(1),
                DCMotor.getKrakenX60(1),
                1.4,
                drivetrain.getModuleLocations(),
                drivetrain.getPigeon2(),
                drivetrain.getModules(),
                kFrontLeft, kFrontRight, kBackLeft, kBackRight
            );
        }

        localization = new Localization(drivetrain);

        try {
            pathplannerConfig = RobotConfig.fromGUISettings();

            /* Configure PathPlanner */
            AutoBuilder.configure(
                () -> localization.getCurrentPose(), 
                drivetrain::resetPose, 
                () -> drivetrain.getState().Speeds, 
                this::drivePathPlanner, 
                new PPHolonomicDriveController(
                    kTranslationPID, 
                    kRotationPID
                ), 
                pathplannerConfig, 
                /* Supplier for if the path should be mirrored for the red alliance - will always use blue for origin */
                () -> DriverStation.getAlliance().orElseGet(() -> DriverStation.Alliance.Blue) == DriverStation.Alliance.Red, 
                this
            );
        } catch (Exception e) {
            ElasticUtil.sendError("Failed to load PathPlanner config", "Auto will commit die!");
        }
        
        /* Allow drive motor constants to be updated from the dashboard */
        SmartDashboard.putNumber("Drive kP", 0.0);
        SmartDashboard.putNumber("Drive kI", 0.0);
        SmartDashboard.putNumber("Drive kD", 0.0);
        SmartDashboard.putNumber("Drive kV", 0.0);
        SmartDashboard.putNumber("Drive kA", 0.0);
        SmartDashboard.putNumber("Drive kS", 0.0);

        SmartDashboard.putData(
            Commands.runOnce(this::updateDriveMotorConstants, this)
            .withName("Update Drive Constants")
            .ignoringDisable(true)
        );

        sysId = new SwerveSysId(this);
    }

    /** Sets the current robot's rotation as the operator perspective */
    public Command resetPerspective() {
        return runOnce(() -> drivetrain.setOperatorPerspectiveForward(drivetrain.getState().Pose.getRotation()));
    }

    public void setControl(SwerveRequest request) {
        drivetrain.setControl(request);
    }

    public Command runSysIdDynamic(Direction direction) {
        return sysId.sysIdDynamic(direction);
    }

    public Command runSysIdQuasistatic(Direction direction) {
        return sysId.sysIdQuasistatic(direction);
    }

    public void drivePathPlanner(ChassisSpeeds setpoint, DriveFeedforwards feedforwards) {
        drivetrain.setControl(
            kRobotRelativeClosedLoopDriveRequest.withSpeeds(setpoint)
            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX())
            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY())
        );
    }

    private void updateDriveMotorConstants() {
        ElasticUtil.sendInfo("Updated constants");

        var modules = drivetrain.getModules();

        TalonFX[] driveMotors = new TalonFX[4];
        for(int i = 0; i < 4; i++) {
            driveMotors[i] = modules[i].getDriveMotor();
        }

        Slot0Configs newConfig = new Slot0Configs()
            .withKP(SmartDashboard.getNumber("Drive kP", 0.0))
            .withKI(SmartDashboard.getNumber("Drive kI", 0.0))
            .withKD(SmartDashboard.getNumber("Drive kD", 0.0))
            .withKV(SmartDashboard.getNumber("Drive kV", 0.0))
            .withKA(SmartDashboard.getNumber("Drive kA", 0.0))
            .withKS(SmartDashboard.getNumber("Drive kS", 0.0));

        for(var motor : driveMotors) {
            ElasticUtil.checkStatus(motor.getConfigurator().apply(newConfig));
        }
    }

    @Override
    public void periodic() {
        localization.updateLocalization();

        if (!operatorPerspectiveSet && DriverStation.getAlliance().isPresent()) {
            boolean isBlue = DriverStation.Alliance.Blue.equals(DriverStation.getAlliance().get());

            drivetrain.setOperatorPerspectiveForward(isBlue ? Rotation2d.kZero : Rotation2d.k180deg);
            
            operatorPerspectiveSet = true;
        }
    }

    @Override
    public void simulationPeriodic() {
        simulatedDrivetrain.update();   

        localization.updateSimulation(simulatedDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose());
    }

    public Localization getLocalization() {
        return localization;
    }

    public Rotation2d getOperatorPerspective() {
        return drivetrain.getOperatorForwardDirection();
    }

    public RobotConfig getPathplannerConfig() {
        return pathplannerConfig;
    }
}