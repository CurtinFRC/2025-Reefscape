package org.curtinfrc.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double appliedVolts;
    public double currentAmps;
    public double encoderOutput;
    // double intakeVolts = IntakeConstants.intakeVolts;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setIntakeVolts(double volts) {}

  public default void intake() {}

  public default void stop() {}
}
