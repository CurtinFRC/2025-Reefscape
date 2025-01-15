package org.curtinfrc.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private DCMotorSim intakeSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getCIM(1), 0.025, 4.0), DCMotor.getCIM(1));

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeSim.setInputVoltage(appliedVolts);
    intakeSim.update(0.02);
    // inputs.appliedVolts = appliedVolts;
    inputs.appliedVolts = intakeSim.getInputVoltage();
  }

  @Override
  public void setIntakeVolts(double volts) {
    appliedVolts = MathUtil.clamp(volts, 12.0, -12.0);
    intakeSim.setInputVoltage(volts);
  }
}
