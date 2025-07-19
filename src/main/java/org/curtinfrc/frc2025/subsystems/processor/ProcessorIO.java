package org.curtinfrc.frc2025.subsystems.processor;

import org.littletonrobotics.junction.AutoLog;

public interface ProcessorIO {
  @AutoLog
  public static class ProcessorIOInputs {}

  public default void updateInputs(ProcessorIOInputs inputs) {}

  public default void setVoltage(double volts) {}
}
