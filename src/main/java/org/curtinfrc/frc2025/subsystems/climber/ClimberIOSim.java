package org.curtinfrc.frc2025.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIOSim implements ClimberIO {
  private DCMotorSim climberMotorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getCIM(1), 0.025, 4.0), DCMotor.getCIM(1));

  private double climberAppliedVoltage = 0.0;

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    climberMotorSim.update(0.02);
    inputs.climberAppliedVoltage = climberMotorSim.getInputVoltage();
    inputs.climberCurrent = climberMotorSim.getCurrentDrawAmps();
  }

  @Override
  public void setClimberVoltage(double voltage) {
    climberAppliedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
    climberMotorSim.setInputVoltage(voltage);
  }
}
