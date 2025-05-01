package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SwerveSysId {
    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    public final SysIdRoutine sysIdRoutineTranslation;

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    public final SysIdRoutine sysIdRoutineSteer;

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    public final SysIdRoutine sysIdRoutineRotation;

    /* The SysId routine to test */
    private final SysIdRoutine sysIdRoutineToApply;

    public SwerveSysId(KrakenSwerve swerve) {
        sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,// Use default ramp rate (1 V/s)
                Volts.of(4),// Reduce dynamic step voltage to 4 V to prevent brownout
                null,// Use default timeout (10 s)
                // Log state with SignalLogger class
                state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                output -> swerve.setControl(translationCharacterization.withVolts(output)),
                null,
                swerve
            )
        );

        sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,//Use default ramp rate (1 V/s)
                Volts.of(7),//Use dynamic voltage of 7 V
                null,//Use default timeout (10 s)
                //Log state with SignalLogger class
                state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                volts -> swerve.setControl(steerCharacterization.withVolts(volts)),
                null,
                swerve
            )
        );

        sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                /* This is in radians per second², but SysId only supports "volts per second" */
                Volts.of(Math.PI / 6).per(Second),
                /* This is in radians per second, but SysId only supports "volts" */
                Volts.of(Math.PI),
                null,//Use default timeout (10 s)
                // Log state with SignalLogger class
                state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                output -> {
                    /* Output is actually radians per second, but SysId only supports "volts" */
                    swerve.setControl(rotationCharacterization.withRotationalRate(output.in(Volts)));
                    /* Log the requested output for SysId */
                    SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                },
                null,
                swerve
            )
        );

        /* Set the routine to run here */
        sysIdRoutineToApply = sysIdRoutineRotation;
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return startLogging()
            .andThen(sysIdRoutineToApply.quasistatic(direction))
            .finallyDo(this::stopLogging);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return startLogging()
            .andThen(sysIdRoutineToApply.dynamic(direction))
            .finallyDo(this::stopLogging);
    }

    private Command startLogging() {
        return Commands.runOnce(() -> SignalLogger.start());
    }

    private Command stopLogging() {
        return Commands.runOnce(() -> SignalLogger.stop());
    }
}   
