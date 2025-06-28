package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.localization.Localization;
import frc.robot.localization.TrackedObject;
import frc.robot.localization.TrackedObject.ObjectType;
import frc.robot.subsystems.swerve.KrakenSwerve;

import static frc.robot.Constants.ObjectAimingConstants.*;
import static frc.robot.Constants.PathFollowingConstants.kRobotRelativeClosedLoopDriveRequest;
import static frc.robot.Constants.DriverConstants.kMaxSpeed;

public class DriverAssistedAutoIntake extends Command {
    private final KrakenSwerve swerve;
    private final DoubleSupplier xSpeedSupplier;
    private final DoubleSupplier ySpeedSupplier;
    private final Localization localization;

    private final PIDController rotationController;

    private TrackedObject lastCoralTarget;

    public DriverAssistedAutoIntake(DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier, KrakenSwerve swerve) {
        this.swerve = swerve;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        
        localization = swerve.getLocalization();

        rotationController = new PIDController(kPRotation, 0, kDRotation);
        rotationController.enableContinuousInput(-180.0, 180.0);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        double currentYaw = swerve.getLocalization().getCurrentPose().getRotation().getDegrees();

        /* Prevent rotating on init if no target is seen */
        rotationController.setSetpoint(currentYaw);
    }

    @Override
    public void execute() {
        double currentYaw = swerve.getLocalization().getCurrentPose().getRotation().getDegrees();

        var coralTarget = getCoralTarget();

        /* Update setpoint if we have a target */
        if (coralTarget.isPresent()) {
            lastCoralTarget = coralTarget.get();

            double rotationSetpoint = currentYaw - (coralTarget.get().getPhotonVisionData().yaw - kRotationSetpoint);

            rotationController.setSetpoint(MathUtil.inputModulus(rotationSetpoint, -180.0, 180.0));
        }

        double xSpeed = xSpeedSupplier.getAsDouble() * kMaxSpeed;
        double ySpeed = ySpeedSupplier.getAsDouble() * kMaxSpeed;

        double magnitude = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

        /* Gets the robot relative speeds as if we are driving normally */
        var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, 
            ySpeed,
            rotationController.calculate(currentYaw),//Apply the rotation PID's output
            localization.getCurrentPose().getRotation()
                .minus(swerve.getOperatorPerspective())//Apply the operator perspective
        );

        /* Only drive straight */
        speeds.vxMetersPerSecond = magnitude * Math.signum(speeds.vxMetersPerSecond);//Use the sign to determine if we are going forwards or backwards
        speeds.vyMetersPerSecond = 0;

        swerve.setControl(kRobotRelativeClosedLoopDriveRequest.withSpeeds(speeds));
    }

    private Optional<TrackedObject> getCoralTarget() {
        /* Persistently track coral targets */
        if (lastCoralTarget != null) {
            var optionalCoral = localization.getTrackedObjectWithTrackingID(lastCoralTarget.getTrackingId());

            if (optionalCoral.isPresent()) 
                return optionalCoral;
        }

        return localization.getNearestObjectOfType(ObjectType.CORAL);
    }

    @Override
    public void end(boolean interrupted) {
        rotationController.reset();

        lastCoralTarget = null;
    }
}