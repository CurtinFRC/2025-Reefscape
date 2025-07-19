package org.curtinfrc.frc2025.subsystems.processor;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ProcessorIOSim implements ProcessorIO {
  private final DCMotor processorMotor = DCMotor.getNEO(1);
  private final DCMotorSim processorMotorSim;
  private final SimDevice frontImpl;
  private final SimBoolean frontSensor;
  private final SimDevice backImpl;
  private final SimBoolean backSensor;
  private double volts = 0;

  public ProcessorIOSim() {
    processorMotorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(processorMotor, 3, 1), processorMotor);

    frontImpl = SimDevice.create("ProcessorSensorFront", 3);
    frontSensor = frontImpl.createBoolean("IsTriggered", Direction.kInput, false);
    backImpl = SimDevice.create("ProcessorSensorBack", 4);
    backSensor = backImpl.createBoolean("IsTriggered", Direction.kInput, false);
  }

  @Override
  public void updateInputs(ProcessorIOInputs inputs) {
    processorMotorSim.setInputVoltage(volts);
    processorMotorSim.update(0.02);
    // inputs.appliedVolts = processorMotorSim.getInputVoltage();
    // inputs.currentAmps = processorMotorSim.getCurrentDrawAmps();
    // inputs.positionRotations = processorMotorSim.getAngularPositionRotations();
    // inputs.angularVelocityRotationsPerMinute = processorMotorSim.getAngularVelocityRPM();
    // inputs.frontSensor = frontSensor.get();
    // inputs.backSensor = backSensor.get();
  }

  @Override
  public void setVoltage(double volts) {
    this.volts = volts;
  }
}
