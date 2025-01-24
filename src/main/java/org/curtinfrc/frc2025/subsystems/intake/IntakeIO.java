package org.curtinfrc.frc2025.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double appliedVolts;
    public double currentAmps;
    // public AngularVelocity encoderOutput;
    public double encoderOutput;
    public double goalRPM = 500;
    public boolean intakeAtRPM = false;
    public boolean beamBreakBroken = false;
    // double intakeVolts = IntakeConstants.intakeVolts;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void achieveRPM() {}

  public default boolean intakeAtRPM() {
    return false;
  }

  public default void beamBreakState() {}

  public default void setIntakeVolts(double volts) {}

  public default void intake() {}

  public default void stop() {}
}
