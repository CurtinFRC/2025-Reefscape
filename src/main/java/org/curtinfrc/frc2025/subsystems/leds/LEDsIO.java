package org.curtinfrc.frc2025.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

public interface LEDsIO {
  @AutoLog
  public static class LEDsIOInputs {
    LEDsColour currentColour = LEDsColour.PINK;
  }

  public default void updateInputs(LEDsIOInputs inputs) {}
  ;

  public enum LEDsColour {
    PINK,
    GREEN,
    BLUE
  };

  public default void switchColor(LEDsColour newColour) {}
}
