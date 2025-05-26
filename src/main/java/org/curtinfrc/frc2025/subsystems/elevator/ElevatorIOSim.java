package org.curtinfrc.frc2025.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim elevatorSim =
      new ElevatorSim(3.35, 0.02, DCMotor.getKrakenX60Foc(2), 0, 0.6, true, 0.01);
  private double volts = 0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    elevatorSim.update(0.02);
    // ignore velocity cause we dont care
    inputs.currentAmps = elevatorSim.getCurrentDrawAmps();
    inputs.appliedVolts = volts;
    inputs.positionRotations = elevatorSim.getPositionMeters();
  }

  @Override
  public void setVoltage(double volts) {
    this.volts = volts;
    elevatorSim.setInputVoltage(volts);
  }
}
