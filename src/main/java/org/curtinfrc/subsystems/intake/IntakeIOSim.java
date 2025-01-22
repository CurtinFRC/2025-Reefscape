package org.curtinfrc.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {

  private DCMotor intakeMotor = DCMotor.getNEO(1);
  private DCMotorSim intakeMotorSim;

  public IntakeIOSim() {
    intakeMotorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(intakeMotor, 0.025, 4.0), intakeMotor);
  }

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeMotorSim.setInputVoltage(appliedVolts);
    intakeMotorSim.update(0.02);
    // inputs.appliedVolts = appliedVolts;
    inputs.appliedVolts = intakeMotorSim.getInputVoltage();
    // inputs.encoderOutput = intakeMotorSim.getAngularVelocity();
    inputs.encoderOutput = intakeMotorSim.getAngularVelocityRadPerSec();
  }

  @Override
  public void setIntakeVolts(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    intakeMotorSim.setInputVoltage(volts);
  }

  @Override
  public void achieveRPM() {
    intakeMotorSim.setAngularVelocity(IntakeConstants.goalRPM);
  }

  @Override
  public boolean intakeAtRPM() {
    double angularVelocity = intakeMotorSim.getAngularVelocityRadPerSec();
    return IntakeConstants.goalRPM - IntakeConstants.intakeTolerance < angularVelocity
        && angularVelocity < IntakeConstants.goalRPM + IntakeConstants.intakeTolerance;
  }
}
