package org.curtinfrc.frc2025.subsystems.popper;

import org.littletonrobotics.junction.AutoLog;

public interface PopperIO {
  @AutoLog
  public static class PopperIOInputs {
    public double appliedVolts;
    public double currentAmps;
    public double positionRotations;
    public double angularVelocityRotationsPerMinute;
  }

  public default void updateInputs(PopperIOInputs inputs) {}

  public default void setVoltage(double volts) {}
}
