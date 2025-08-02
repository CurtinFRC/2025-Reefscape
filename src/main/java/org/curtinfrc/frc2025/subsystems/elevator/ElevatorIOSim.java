package org.curtinfrc.frc2025.subsystems.elevator;

import static org.curtinfrc.frc2025.subsystems.elevator.ElevatorConstants.kA;
import static org.curtinfrc.frc2025.subsystems.elevator.ElevatorConstants.kV;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim elevatorSim =
      new ElevatorSim(kV, kA, DCMotor.getNEO(1), 0, 0.6, true, 0);
  private double volts = 0;

  private final SimDevice safeImpl;
  private final SimBoolean safeSensor;

  public ElevatorIOSim() {
    safeImpl = SimDevice.create("ElevatorSafe", 4);
    safeSensor = safeImpl.createBoolean("IsSafe", Direction.kInput, false);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    elevatorSim.update(0.02);
    // ignore velocity cause we dont care
    inputs.currentAmps = elevatorSim.getCurrentDrawAmps();
    inputs.appliedVolts = volts;
    inputs.positionRotations = elevatorSim.getPositionMeters();
    inputs.safe = safeSensor.get();
  }

  @Override
  public void setVoltage(double volts) {
    this.volts = volts;
    elevatorSim.setInputVoltage(volts);
  }
}
