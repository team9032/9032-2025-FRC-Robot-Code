package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.automation.ButtonBoardHandler.ReefLevel;

import static frc.robot.subsystems.leds.LEDConstants.*;

import java.util.Map;
import java.util.function.Supplier;

public class LEDs extends SubsystemBase {
    public static enum State {
        LOW_BATTERY(kBatteryLowPattern),
        BOOTING(kBootingUp),
        DISABLED(kDisabledPattern),
        ENABLED_L1(kEnabledL1Pattern),
        ENABLED_L2(kEnabledL2Pattern),
        ENABLED_L3(kEnabledL3Pattern),
        ENABLED_L4(kEnabledL4Pattern),
        SCORING_L1(kScoringL1Pattern),
        SCORING_L2(kScoringL2Pattern),
        SCORING_L3(kScoringL3Pattern),
        SCORING_L4(kScoringL4Pattern),
        ALGAE(kAlgaePattern),
        ERROR(kError),
        CLIMBING(kClimbing);

        public final LEDPattern statePattern;

        State(LEDPattern statePattern) {
            this.statePattern = statePattern;
        }
    }

    private boolean hasError = false;

    private State currentState = State.BOOTING;//Default to booting - gets changed to disable when booted

    private final AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;

    public LEDs() {
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

    public void displayError() {
        hasError = true;
    }

    public Command setEnabledStateFromReefLevel(Supplier<ReefLevel> reefLevelSup) {
        return new SelectCommand<ReefLevel>(
            Map.ofEntries (
                Map.entry(ReefLevel.NONE, Commands.none()),
                Map.entry(ReefLevel.L1, setStateCommand(State.ENABLED_L1)),
                Map.entry(ReefLevel.L2, setStateCommand(State.ENABLED_L2)),
                Map.entry(ReefLevel.L3, setStateCommand(State.ENABLED_L3)),
                Map.entry(ReefLevel.L4, setStateCommand(State.ENABLED_L4))
            ),
            reefLevelSup
        );
    } 

    public Command setScoringStateFromReefLevel(Supplier<ReefLevel> reefLevelSup) {
        return new SelectCommand<ReefLevel>(
            Map.ofEntries (
                Map.entry(ReefLevel.NONE, Commands.none()),
                Map.entry(ReefLevel.L1, setStateCommand(State.SCORING_L1)),
                Map.entry(ReefLevel.L2, setStateCommand(State.SCORING_L2)),
                Map.entry(ReefLevel.L3, setStateCommand(State.SCORING_L3)),
                Map.entry(ReefLevel.L4, setStateCommand(State.SCORING_L4))
            ),
            reefLevelSup
        );
    }

    @Override
    public void periodic() {
        if (hasError)
            applyPattern(State.ERROR.statePattern);

        else
            applyPattern(currentState.statePattern);
    }
}