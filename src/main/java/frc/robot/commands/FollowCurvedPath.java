package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.pathing.CurvedPath;
import frc.robot.subsystems.swerve.KrakenSwerve;
import frc.robot.utils.GeometryUtil;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import static frc.robot.pathing.PathingConstants.*;

import com.ctre.phoenix6.Utils;

public class FollowCurvedPath extends Command {
    private final ProfiledPIDController rotationPID;

    private final KrakenSwerve swerve;

    private final CurvedPath path;
    
    private Translation2d previousVelocity;
    private double previousTime;

    public FollowCurvedPath(KrakenSwerve swerve, CurvedPath path) {
        rotationPID = new ProfiledPIDController(kRotationkP, 0, kRotationkD, kRotationConstraints);
        rotationPID.setTolerance(kRotationAlignmentTolerance.in(Radians));
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);

        this.swerve = swerve;

        this.path = path;      

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = swerve.getLocalization().getCurrentPose();
        ChassisSpeeds currentVelocity = swerve.getLocalization().getCurrentVelocity();

        rotationPID.setGoal(path.finalPose().getRotation().getRadians());
        rotationPID.reset(currentPose.getRotation().getRadians(), currentVelocity.omegaRadiansPerSecond);

        previousVelocity = new Translation2d(currentVelocity.vxMetersPerSecond, currentVelocity.vyMetersPerSecond);
        previousTime = Utils.getCurrentTimeSeconds();
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getLocalization().getCurrentPose();
        ChassisSpeeds currentSpeeds = swerve.getLocalization().getCurrentVelocity();
        Translation2d currentVelocity = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

        var driveDirection = path.getPathDirection(currentPose.getTranslation());
        double remaingPathDistance = path.getRemainingPathDistance(currentPose.getTranslation());

        /* Use kinematics to find the maximum drive speed that can be stopped with the max acceleration */
        double driveSpeed = Math.sqrt(Math.pow(path.endingSpeed(), 2) - (2.0 * kMaxAcceleration * -remaingPathDistance));

        /* Apply speed limit */
        driveSpeed = Math.min(driveSpeed, kMaxSpeed);

        /* Find angular velocity by combining the profiled PID's output and its velocity setpoint */
        double angularVelocity = rotationPID.calculate(currentPose.getRotation().getRadians()) + rotationPID.getSetpoint().velocity;

        /* Limit drive speed based on rotation speed */
        double wheelSpeedFromAngularVelocity = swerve.getDrivebaseRadius() * angularVelocity;

        SmartDashboard.putBoolean("Pathing/Rotation Limited", kTrueMaxSpeed - wheelSpeedFromAngularVelocity < driveSpeed);
        driveSpeed = Math.min(driveSpeed, kTrueMaxSpeed - wheelSpeedFromAngularVelocity);

        /* Find target velocity using drive speed and direction */
        var targetVelocity = driveDirection.times(driveSpeed);

        /* Find target acceleration */
        double currentTime = Utils.getCurrentTimeSeconds();
        double dt = currentTime - previousTime;
        var targetAcceleration = (targetVelocity.minus(previousVelocity)).div(dt);

        double maxForwardAcceleration;
        /* If we are in the torque limited part of the motor curve, limit forward acceleration based on available torque */
        SmartDashboard.putBoolean("Pathing/Torque Limited", currentVelocity.getNorm() >= kTorqueLimitedSpeedStart);
        if (currentVelocity.getNorm() >= kTorqueLimitedSpeedStart) 
            maxForwardAcceleration = kTrueMaxAcceleration * (1.0 - (currentVelocity.getNorm() / kTrueMaxSpeed));

        /* Use the true max acceleration since we are in the current limited part of the motor curves */
        else 
            maxForwardAcceleration = kTrueMaxAcceleration;

        /* Apply forward and reverse acceleration limit */
        SmartDashboard.putBoolean("Pathing/Acceleration Limited", targetAcceleration.getNorm() > maxForwardAcceleration);

        double accelerationMagnitude = Math.min(targetAcceleration.getNorm(), maxForwardAcceleration);
        targetAcceleration = GeometryUtil.normalize(targetAcceleration).times(accelerationMagnitude);
        
        targetVelocity = previousVelocity.plus(targetAcceleration.times(dt));

        swerve.setControl(
            kFieldCentricClosedLoopDriveRequest
                .withVelocityX(targetVelocity.getX())
                .withVelocityY(targetVelocity.getY())
                .withRotationalRate(angularVelocity)
        );

        SmartDashboard.putNumber("Pathing/Target Acceleration Magnitude", targetAcceleration.getNorm());
        SmartDashboard.putNumber("Pathing/Target Speed", driveSpeed);
        SmartDashboard.putNumber("Pathing/Target Angular Velocity", angularVelocity);

        previousVelocity = targetVelocity;
        previousTime = currentTime;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(kRobotRelativeClosedLoopDriveRequest.withSpeeds(new ChassisSpeeds()));
    }

    private boolean atSetpoint() {
        var currentTranslation = swerve.getLocalization().getCurrentPose().getTranslation();
        double currentSpeed = swerve.getLocalization().getCurrentSpeed();

        return rotationPID.atGoal() 
            && currentTranslation.getDistance(path.finalPose().getTranslation()) < kXYAlignmentTolerance.in(Meters)
            && currentSpeed < kAcceptableEndingSpeed;
    }

    @Override
    public boolean isFinished() {
        /* It is not important to be at setpoint if there is a nonzero ending speed since we are transitioning to another drive command */
        if (path.endingSpeed() > 0.0) {
            var currentTranslation = swerve.getLocalization().getCurrentPose().getTranslation();

            return currentTranslation.getDistance(path.finalPose().getTranslation()) < kRoughXYAlignmentTolerance.in(Meters);
        }

        else 
            return atSetpoint();
    }
}