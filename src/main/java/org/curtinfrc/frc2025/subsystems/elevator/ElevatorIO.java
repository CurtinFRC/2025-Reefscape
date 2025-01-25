package org.curtinfrc.frc2025.subsystems.elevator;

import org.curtinfrc.frc2025.Constants.Setpoints;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double distanceSensorReading;
    public double encoderReading;
    public Setpoints point;
    public double pointRot;
    public double motorVoltage;
    public double motorCurrent;
    public double motorTemp;
    public double motorVelocity;
    public double positionError;
    public double velocityError;
    public boolean stable;
    public int predictionHorizon;
    public double predictedPosition;
    public double predictedVelocity;
  }

  public default void goToSetpoint(Setpoints point) {}

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void reset() {}

  public default boolean isStable() {
    return false;
  }

  public default void stop() {}

  public static double convertSetpoint(double set /* in mm */) {
    return (set - ElevatorConstants.height) / 21; /* mm to revolutions */
  }
}
