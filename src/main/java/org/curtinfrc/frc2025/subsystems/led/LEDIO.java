package org.curtinfrc.frc2025.subsystems.led;

import org.littletonrobotics.junction.AutoLog;

public interface LEDIO {
  @AutoLog
  public static class LEDIOInputs {
    public State state = State.Inactive;
  }

  public static enum State {
    Active,
    Blink,
    Inactive,
  }

  public default void updateInputs(LEDIOInputs inputs) {}

  public default void setState(State state) {}
}
