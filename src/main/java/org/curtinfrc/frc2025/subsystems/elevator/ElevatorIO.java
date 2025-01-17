package org.curtinfrc.frc2025.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double distanceSensorReading;
    public double encoderReading;
    public ElevatorConstants.Setpoints point;
    public double motorVoltage;
    public double motorCurrent;
    public double motorTemp;
    public double motorVelocity;
  }

  public default void goToSetpoint(ElevatorConstants.Setpoints point) {}

  public default void applyVoltage(double volts) {}

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void reset() {}

  public default boolean isStable() {
    return false;
  }
}
