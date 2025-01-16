package org.curtinfrc.frc2025.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double pivotAppliedVoltage = 0.0;
    public double pivotCurrent = 0.0;
    public double pivotEncoderPosition = 0.0;

    public double grabberAppliedVoltage = 0.0;
    public double grabberCurrent = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setPivotVoltage(double voltage) {}

  public default void setGrabberVoltage(double voltage) {}

  public default void goToPivotSetpoint() {}
}
