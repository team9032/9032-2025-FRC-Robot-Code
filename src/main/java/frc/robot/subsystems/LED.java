package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.LEDConstants.*;

public class LED extends SubsystemBase {
    public static enum State {
        DISABLED(kDisabledPattern),
        ENABLED(kEnabledPattern),
        L1(kL1Pattern),
        L2(kL2Pattern),
        L3(kL3Pattern),
        L4(kL4Pattern);

        public final LEDPattern statePattern;

        State(LEDPattern statePattern) {
            this.statePattern = statePattern;
        }
    }

    private State currentState = State.DISABLED;
    private final AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;

    public LED() {
        ledStrip = new AddressableLED(kLEDPort);
        ledStrip.setLength(kLEDLength);

        ledBuffer = new AddressableLEDBuffer(kLEDLength);
        
        ledStrip.start();
    }

    public Command setState(State state) {
        return runOnce(() -> currentState = state);
    }

    private void applyPattern(LEDPattern pattern) {
        pattern.applyTo(ledBuffer);
    }

    private void setData() {
        ledStrip.setData(ledBuffer);
    }

    @Override
    public void periodic() {
        applyPattern(currentState.statePattern);
        setData();
    }
}