package org.curtinfrc.frc2025.subsystems.ejector;

import static org.curtinfrc.frc2025.subsystems.ejector.EjectorConstants.*;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class EjectorIOSim implements EjectorIO {
  private DCMotor intakeMotor = DCMotor.getNEO(1);
  private DCMotorSim intakeMotorSim;
  private SimDevice sensorImpl;
  private SimBoolean sensor;
  private double volts = 0;

  public EjectorIOSim() {
    intakeMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(intakeMotor, ejectorMoi, ejectorReduction),
            intakeMotor);

    sensorImpl = SimDevice.create("EjectorSensor", sensorPort);
    sensor = sensorImpl.createBoolean("IsTriggered", Direction.kInput, false);
  }

  @Override
  public void updateInputs(EjectorIOInputs inputs) {
    intakeMotorSim.setInputVoltage(volts);
    intakeMotorSim.update(0.02);
    inputs.appliedVolts = intakeMotorSim.getInputVoltage();
    inputs.currentAmps = intakeMotorSim.getCurrentDrawAmps();
    inputs.positionRotations = intakeMotorSim.getAngularPositionRotations();
    inputs.angularVelocityRotationsPerMinute = intakeMotorSim.getAngularVelocityRPM();
    inputs.sensor = sensor.get();
  }

  @Override
  public void setVoltage(double volts) {
    this.volts = volts;
    intakeMotorSim.setInputVoltage(volts);
  }
}
