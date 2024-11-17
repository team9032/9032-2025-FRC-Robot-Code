package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.SwerveConstants.*;

public class KrakenSwerve extends SubsystemBase {
    public SwerveDrivetrain driveTrain;

    public KrakenSwerve() {
        driveTrain = new SwerveDrivetrain(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
    }
}
