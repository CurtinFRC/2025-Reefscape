package org.curtinfrc.frc2025.subsystems.elevator;

import org.curtinfrc.frc2025.Constants.Setpoints;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double distanceSensorReading;
    public double encoderReading;
    public Setpoints point;
    public double motorVoltage;
    public double motorVelocity;
    public double motorCurrent;
    public double motorTemp;
  }

  public default void goToSetpoint(Setpoints point) {}

  public default void applyVoltage(double volts) {}

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default boolean isStable() {
    return false;
  }
}
