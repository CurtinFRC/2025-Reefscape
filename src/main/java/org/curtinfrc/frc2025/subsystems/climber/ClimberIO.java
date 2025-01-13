package org.curtinfrc.frc2025.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double climberAppliedVoltage = 0.0;
    public double climberCurrent = 0.0;
    public double climberEncoderPosition = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setClimberVoltage(double voltage) {}
}
