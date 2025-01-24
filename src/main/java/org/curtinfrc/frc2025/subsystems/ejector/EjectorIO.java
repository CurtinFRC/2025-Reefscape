package org.curtinfrc.frc2025.subsystems.ejector;

import org.littletonrobotics.junction.AutoLog;

public interface EjectorIO {
  @AutoLog
  public static class EjectorIOInputs {
    public double EjectorGoalRPM;
    public double ejectorEncoderOutput;
    public double appliedVolts;
  }

  public default void updateInputs(EjectorIOInputs inputs) {}

  public default boolean ejectorAtRPM() {
    return false;
  }

  public default void setEjectorVolts(double volts) {}

  public default void achieveEjectorRPM() {}
}
