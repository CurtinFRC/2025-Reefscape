package org.curtinfrc.frc2025.subsystems.processor;

import org.littletonrobotics.junction.AutoLog;

public interface ProcessorIO {
  @AutoLog
  public static class ProcessorIOInputs {
    public double armAppliedVolts;
    public double armCurrentAmps;
    public double armPositionRotations;
    public double armAbsolutePosition;
    public double armAngularVelocityRotationsPerMinute;
    public double intakeAppliedVolts;
    public double intakeCurrentAmps;
    public double intakePositionRotations;
    public double intakeAngularVelocityRotationsPerMinute;
    public boolean processorSensor;
  }

  public default void updateInputs(ProcessorIOInputs inputs) {}

  public default void armSetVoltage(double volts) {}

  public default void intakeSetVoltage(double volts) {}
}
