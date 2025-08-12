package org.curtinfrc.frc2025.subsystems.ejector;

import org.littletonrobotics.junction.AutoLog;

public interface EjectorIO {
  @AutoLog
  public static class EjectorIOInputs {
    public double appliedVolts;
    public double currentAmps;
    public double positionRotations;
    public double angularVelocityRotationsPerSecond;
    public boolean backSensor;
    public boolean frontSensor;
  }

  public default void updateInputs(EjectorIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setVelocity(double speedRotationsPerSecond) {}

  public default double getkP() {
    return 0.0;
  }

  public default void setkP(double kP) {}

  public default double getkV() {
    return 0.0;
  }

  public default void setkV(double kV) {}
}
