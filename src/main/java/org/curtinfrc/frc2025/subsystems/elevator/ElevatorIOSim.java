package org.curtinfrc.frc2025.subsystems.elevator;

import static org.curtinfrc.frc2025.subsystems.elevator.ElevatorConstants.kA;
import static org.curtinfrc.frc2025.subsystems.elevator.ElevatorConstants.kV;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim elevatorSim =
      new ElevatorSim(kV, kA, DCMotor.getNEO(1), 0, 0.6, true, 0);
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
