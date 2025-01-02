package org.curtinfrc.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double appliedVoltage = 0.0;
  }

  public default void intake() {}

  public default void stop() {}

  public default void updateInputs(IntakeIOInputs inputs) {}
}
