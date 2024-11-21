package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KrakenSwerve extends SubsystemBase {
    public final SwerveDrivetrain driveTrain;

    public KrakenSwerve() {
        driveTrain = new SwerveDrivetrain(drivetrainConstants, frontLeft, frontRight, backLeft, backRight);
    }

    public Command zeroGyro() {
        return runOnce(() -> driveTrain.getPigeon2().reset());
    }
}
