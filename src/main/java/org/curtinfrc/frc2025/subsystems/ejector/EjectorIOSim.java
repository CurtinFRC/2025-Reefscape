package org.curtinfrc.frc2025.subsystems.ejector;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class EjectorIOSim implements EjectorIO {
  private final DCMotor ejectorMotor = DCMotor.getNEO(1);
  private final DCMotorSim ejectorMotorSim;
  private final SimDevice frontImpl;
  private final SimBoolean frontSensor;
  private final SimDevice backImpl;
  private final SimBoolean backSensor;
  private double volts = 0;

  public EjectorIOSim() {
    ejectorMotorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(ejectorMotor, 3, 1), ejectorMotor);

    frontImpl = SimDevice.create("EjectorSensorFront", 1);
    frontSensor = frontImpl.createBoolean("IsTriggered", Direction.kInput, true);
    backImpl = SimDevice.create("EjectorSensorBack", 2);
    backSensor = backImpl.createBoolean("IsTriggered", Direction.kInput, true);
  }

  @Override
  public void updateInputs(EjectorIOInputs inputs) {
    ejectorMotorSim.setInputVoltage(volts);
    ejectorMotorSim.update(0.02);
    inputs.appliedVolts = ejectorMotorSim.getInputVoltage();
    inputs.currentAmps = ejectorMotorSim.getCurrentDrawAmps();
    inputs.positionRotations = ejectorMotorSim.getAngularPositionRotations();
    inputs.angularVelocityRotationsPerMinute = ejectorMotorSim.getAngularVelocityRPM();
    inputs.frontSensor = frontSensor.get();
    inputs.backSensor = backSensor.get();
  }

  @Override
  public void setVoltage(double volts) {
    this.volts = volts;
  }
}
