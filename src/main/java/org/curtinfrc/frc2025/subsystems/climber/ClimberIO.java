package org.curtinfrc.frc2025.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double grabberAppliedVoltage = 0.0;
    public double grabberCurrent = 0.0;
    public double grabberEncoderPosition = 0.0;

    public boolean grabberIsStable = false;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setGrabberVoltage(double voltage) {}

  public default void goToGrabberSetpoint() {}

  public default boolean grabberIsStable() {
    return false;
  }
}
