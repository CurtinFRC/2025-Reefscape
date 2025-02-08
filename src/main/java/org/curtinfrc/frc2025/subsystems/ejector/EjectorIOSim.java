package org.curtinfrc.frc2025.subsystems.ejector;

import static org.curtinfrc.frc2025.subsystems.ejector.EjectorConstants.*;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class EjectorIOSim implements EjectorIO {
  private DCMotor ejectorMotor = DCMotor.getNEO(1);
  private DCMotorSim ejectorMotorSim;
  private SimDevice sensorImpl;
  private SimBoolean sensor;
  private double volts = 0;

  public EjectorIOSim() {
    ejectorMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(ejectorMotor, ejectorMoi, ejectorReduction),
            ejectorMotor);

    sensorImpl = SimDevice.create("EjectorSensor", sensorPort);
    sensor = sensorImpl.createBoolean("IsTriggered", Direction.kInput, false);
  }

  @Override
  public void updateInputs(EjectorIOInputs inputs) {
    ejectorMotorSim.setInputVoltage(volts);
    ejectorMotorSim.update(0.02);
    inputs.appliedVolts = ejectorMotorSim.getInputVoltage();
    inputs.currentAmps = ejectorMotorSim.getCurrentDrawAmps();
    inputs.positionRotations = ejectorMotorSim.getAngularPositionRotations();
    inputs.angularVelocityRotationsPerMinute = ejectorMotorSim.getAngularVelocityRPM();
    inputs.sensor = sensor.get();
  }

  @Override
  public void setVoltage(double volts) {
    this.volts = volts;
    ejectorMotorSim.setInputVoltage(volts);
  }
}
