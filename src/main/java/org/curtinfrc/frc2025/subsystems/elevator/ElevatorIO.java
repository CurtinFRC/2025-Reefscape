package org.curtinfrc.frc2025.subsystems.elevator;

import org.curtinfrc.frc2025.Constants.Setpoint;
import org.curtinfrc.frc2025.subsystems.elevator.ElevatorConstants.ElevatorSetpoints;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double appliedVolts;
    public double currentAmps;
    public double positionRotations;
    public double angularVelocityRotationsPerMinute;
    public boolean hominSensor;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void zero() {}

  public default void goToSetpoint(ElevatorSetpoints point) {}

  public default double positionRotationsToMetres(double rotations) {
    return rotations;
  }

  public default double positionMetresToRotations(double metres) {
    return metres;
  }
}
