package org.curtinfrc.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double appliedVolts;
    public double currentAmps;
    // public AngularVelocity encoderOutput;
    public double encoderOutput;
    public double goalRPM = 5000;
    public boolean intakeAtRPM = false;
    // double intakeVolts = IntakeConstants.intakeVolts;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void achieveRPM(double goalRPM) {}

  public default boolean intakeAtRPM() {
    return false;
  }

  public default void setIntakeVolts(double volts) {}

  public default void intake() {}

  public default void stop() {}
}
