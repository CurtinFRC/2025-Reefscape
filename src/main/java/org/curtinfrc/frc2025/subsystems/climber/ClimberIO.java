package org.curtinfrc.frc2025.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double appliedVoltage;
    public double currentAmps;
    public double positionRotations;
    public double angularVelocityRotationsPerMinute;
    public boolean atSetpoint;
    public double ratchet;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void goToSetpoint() {}

  public default void engageRatchet() {}

  public default void disableRatchet() {}
}
