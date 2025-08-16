package org.curtinfrc.frc2025.subsystems.popper;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class PopperIOSim implements PopperIO {
  private final DCMotor popperMotor = DCMotor.getNEO(1);
  private final DCMotorSim popperMotorSim;
  private double volts = 0;

  public PopperIOSim() {
    popperMotorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(popperMotor, 3, 1), popperMotor);
  }

  @Override
  public void updateInputs(PopperIOInputs inputs) {
    popperMotorSim.setInputVoltage(volts);
    popperMotorSim.update(0.02);
    inputs.appliedVolts = popperMotorSim.getInputVoltage();
    inputs.currentAmps = popperMotorSim.getCurrentDrawAmps();
    inputs.positionRotations = popperMotorSim.getAngularPositionRotations();
  }

  @Override
  public void setVoltage(double volts) {
    this.volts = volts;
  }

  @Override
  public void setVelocity(double speedRotationsPerSecond) {
    popperMotorSim.setAngularVelocity(speedRotationsPerSecond * Math.PI);
  }
}
