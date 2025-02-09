package org.curtinfrc.frc2025.subsystems.ejector;

import org.littletonrobotics.junction.AutoLog;

public interface EjectorIO {
  @AutoLog
  public static class EjectorIOInputs {
    public double appliedVolts;
    public double currentAmps;
    public double positionRotations;
    public double angularVelocityRotationsPerMinute;
    public boolean sensor;
  }

  public default void updateInputs(EjectorIOInputs inputs) {}

  public default void setVoltage(double volts) {}
}
