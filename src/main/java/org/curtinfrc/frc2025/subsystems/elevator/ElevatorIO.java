package org.curtinfrc.frc2025.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double distanceSensorReading;
    public double encoderReading;
    public double position;
    public ElevatorConstants.Setpoints point;
  }

  public default void goToSetpoint(ElevatorConstants.Setpoints point) {}

  public default void applyVoltage(double volts) {}

  public default void updateInputs(ElevatorIOInputs inputs) {}
}
