package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.LEDConstants.*;

public class LED extends SubsystemBase {
    public static enum State {
        RAINBOW, BLUE_GRADIENT, BLUE_BREATHE, GREEN
    }

    private State currentState = State.RAINBOW;
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

    private void setRainbow() {
        applyPattern(scrollingRainbow);
        setData();
    }

    private void setBlueGradient() {
        applyPattern(blueGradient);
        setData();
    }

    private void setBlueBreathe() {
        applyPattern(blueBreathe);
        setData();
    }

    private void setGreen() {
        applyPattern(green);
        setData();
    }

    private void applyPattern(LEDPattern pattern) {
        pattern.applyTo(ledBuffer);
    }

    private void setData() {
        ledStrip.setData(ledBuffer);
    }

    @Override
    public void periodic() {
        switch (currentState) {
            case RAINBOW:
                setRainbow();
            case BLUE_GRADIENT:
                setBlueGradient();
            case BLUE_BREATHE:
                setBlueBreathe();
            case GREEN:
                setGreen();
        }
    }
}