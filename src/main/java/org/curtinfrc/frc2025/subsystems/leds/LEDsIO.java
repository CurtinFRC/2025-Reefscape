package org.curtinfrc.frc2025.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

public interface LEDsIO {
    @AutoLog
    public static class LEDsIOInputs {
        LEDsColour currentColour = LEDsColour.PINK;
        LEDsState currentState = LEDsState.STATIC;
    }

    public default void updateInputs(LEDsIOInputs inputs) {};
    public enum LEDsState {STATIC, BLINK};
    public enum LEDsColour {PINK, GREEN};

    public default void switchColor(LEDsColour newColour) {}
    public default void switchState(LEDsState newState) {}
}