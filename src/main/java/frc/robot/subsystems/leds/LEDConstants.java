package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;

public final class LEDConstants {
    public static final int kLEDPort = 0;
    public static final int kLEDLength = 99;
    public static final Distance kLedSpacing = Meters.of(1.0 / 120.0);

    //Patterns
    public static final LEDPattern kError = LEDPattern.solid(Color.kDarkRed);

    public static final LEDPattern kClimbing = LEDPattern.rainbow(255, 255);

    public static final LEDPattern kBootingUp = LEDPattern.solid(Color.kPurple);

    public static final LEDPattern kDisabledPattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kDarkBlue, Color.kBlue, Color.kAquamarine)
        .scrollAtAbsoluteSpeed(InchesPerSecond.of(8.0), kLedSpacing)
        .breathe(Seconds.of(2));

    public static final LEDPattern kEnabledL1Pattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kWhite, Color.kBlack)
        .scrollAtAbsoluteSpeed(InchesPerSecond.of(20.0), kLedSpacing)
        .synchronizedBlink(() -> RobotController.getRSLState());

    public static final LEDPattern kEnabledL2Pattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kYellow, Color.kLightGoldenrodYellow)
        .scrollAtAbsoluteSpeed(InchesPerSecond.of(20.0), kLedSpacing)
        .synchronizedBlink(() -> RobotController.getRSLState());

    public static final LEDPattern kEnabledL3Pattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kDarkOliveGreen, Color.kDarkGreen)
        .scrollAtAbsoluteSpeed(InchesPerSecond.of(20.0), kLedSpacing)
        .synchronizedBlink(() -> RobotController.getRSLState());

    public static final LEDPattern kEnabledL4Pattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kDarkBlue, Color.kBlue, Color.kAquamarine)
        .scrollAtAbsoluteSpeed(InchesPerSecond.of(20.0), kLedSpacing)
        .synchronizedBlink(() -> RobotController.getRSLState());

    public static final LEDPattern kScoringL1Pattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kWhite, Color.kBlack)
        .scrollAtAbsoluteSpeed(InchesPerSecond.of(100.0), kLedSpacing);

    public static final LEDPattern kScoringL2Pattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kYellow, Color.kLightGoldenrodYellow)
        .scrollAtAbsoluteSpeed(InchesPerSecond.of(100.0), kLedSpacing);

    public static final LEDPattern kScoringL3Pattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kDarkOliveGreen, Color.kDarkGreen)
        .scrollAtAbsoluteSpeed(InchesPerSecond.of(100.0), kLedSpacing);

    public static final LEDPattern kScoringL4Pattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kDarkBlue, Color.kBlue, Color.kAquamarine)
        .scrollAtAbsoluteSpeed(InchesPerSecond.of(100.0), kLedSpacing);

    public static final LEDPattern kAlgaePattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous,  Color.kDarkBlue, Color.kPurple, Color.kDarkViolet)
        .scrollAtAbsoluteSpeed(InchesPerSecond.of(100.0), kLedSpacing);

    public static final LEDPattern kBatteryLowPattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kDarkRed, Color.kOrange)
        .scrollAtAbsoluteSpeed(InchesPerSecond.of(8.0), kLedSpacing)
        .blink(Seconds.of(0.6));
}