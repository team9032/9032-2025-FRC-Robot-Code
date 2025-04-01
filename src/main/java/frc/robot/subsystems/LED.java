package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.LEDConstants.*;

public class LED extends SubsystemBase {
    public static enum State {
        LOW_BATTERY(kBatteryLowPattern),
        BOOTING(kBootingUp),
        DISABLED(kDisabledPattern),
        ENABLED(kEnabledPattern),
        L1(kL1Pattern),
        L2(kL2Pattern),
        L3(kL3Pattern),
        L4(kL4Pattern),
        ALGAE(kAlgaePattern),
        ERROR(kError);

        public final LEDPattern statePattern;

        State(LEDPattern statePattern) {
            this.statePattern = statePattern;
        }
    }

    private State currentState = State.BOOTING;//Default to booting - gets changed to disable when booted

    private final AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;

    public LED() {
        ledStrip = new AddressableLED(kLEDPort);
        ledStrip.setLength(kLEDLength);

        ledBuffer = new AddressableLEDBuffer(kLEDLength);
        
        ledStrip.start();

        /* Set booting state */
        applyPattern(kBootingUp);
    }

    public Command setStateCommand(State state) {
        return Commands.runOnce(() -> setState(state))//The use of Commands.runOnce ensures that this command does not require this subsystem
            .ignoringDisable(true);
    }

    public void setState(State state) {
        currentState = state;
    }

    private void applyPattern(LEDPattern pattern) {
        pattern.applyTo(ledBuffer);

        ledStrip.setData(ledBuffer);
    }

    @Override
    public void periodic() {
        applyPattern(currentState.statePattern);
    }
}