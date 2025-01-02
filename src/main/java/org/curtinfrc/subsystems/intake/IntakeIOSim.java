package org.curtinfrc.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private DCMotor motor = DCMotor.getKrakenX60Foc(1);
  private DCMotorSim intakeSim;

  public IntakeIOSim() {
    intakeSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 0.025, 4.0), motor);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeSim.update(0.02);
  }

  @Override
  public void intake() {
    intakeSim.setInputVoltage(IntakeConstants.intakeVolts);
  }

  @Override
  public void stop() {
    intakeSim.setInputVoltage(0.0);
  }
}
