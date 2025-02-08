package org.curtinfrc.frc2025.subsystems.elevator;

import static org.curtinfrc.frc2025.subsystems.elevator.ElevatorConstants.heightMetersToRotations;
import static org.curtinfrc.frc2025.subsystems.elevator.ElevatorConstants.massKgs;
import static org.curtinfrc.frc2025.subsystems.elevator.ElevatorConstants.radiusMeters;
import static org.curtinfrc.frc2025.subsystems.elevator.ElevatorConstants.resetPort;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private DCMotor elevatorMotor = DCMotor.getNEO(1);
  private ElevatorSim elevatorSim;
  private SimDevice sensorImpl;
  private SimBoolean sensor;
  private double volts = 0;

  public ElevatorIOSim() {
    elevatorSim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(elevatorMotor, massKgs, radiusMeters, 1),
            elevatorMotor,
            0,
            2,
            true,
            0);

    sensorImpl = SimDevice.create("EjectorSensor", resetPort);
    sensor = sensorImpl.createBoolean("IsTriggered", Direction.kInput, false);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    elevatorSim.setInputVoltage(volts);
    elevatorSim.update(0.02);
    // ignore velocity
    inputs.appliedVolts = elevatorSim.getInput(0);
    inputs.currentAmps = elevatorSim.getCurrentDrawAmps();
    inputs.positionRotations = heightMetersToRotations(elevatorSim.getPositionMeters());
    inputs.touchSensor = sensor.get();
  }

  @Override
  public void setVoltage(double volts) {
    this.volts = volts;
    elevatorSim.setInputVoltage(volts);
  }
}
