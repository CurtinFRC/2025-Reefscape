package org.curtinfrc.frc2025.subsystems.processor;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ProcessorIOSim implements ProcessorIO {
  private final DCMotor processorArmMotor = DCMotor.getNEO(1);
  private final DCMotorSim processorArmMotorSim;
  private final DCMotor processorIntakeMotor = DCMotor.getNEO(1);
  private final DCMotorSim processorIntakeMotorSim;
  private final SimDevice sensorImpl;
  private final SimBoolean processorSensor;
  private double armVolts = 0;
  private double intakeVolts = 0;

  public ProcessorIOSim() {
    processorArmMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(processorArmMotor, 3, 1), processorArmMotor);
    processorIntakeMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(processorIntakeMotor, 3, 1), processorIntakeMotor);

    sensorImpl = SimDevice.create("ProcessorSensorFront", 3);
    processorSensor = sensorImpl.createBoolean("IsTriggered", Direction.kInput, false);
  }

  @Override
  public void updateInputs(ProcessorIOInputs inputs) {
    processorArmMotorSim.setInputVoltage(armVolts);
    processorArmMotorSim.update(0.02);
    processorIntakeMotorSim.setInputVoltage(intakeVolts);
    processorIntakeMotorSim.update(0.02);
    inputs.armAppliedVolts = processorArmMotorSim.getInputVoltage();
    inputs.armCurrentAmps = processorArmMotorSim.getCurrentDrawAmps();
    inputs.armPositionRotations = processorArmMotorSim.getAngularPositionRotations();
    inputs.armAngularVelocityRotationsPerMinute = processorArmMotorSim.getAngularVelocityRPM();
    inputs.intakeAppliedVolts = processorIntakeMotorSim.getInputVoltage();
    inputs.intakeCurrentAmps = processorIntakeMotorSim.getCurrentDrawAmps();
    inputs.intakePositionRotations = processorIntakeMotorSim.getAngularPositionRotations();
    inputs.intakeAngularVelocityRotationsPerMinute =
        processorIntakeMotorSim.getAngularVelocityRPM();
    inputs.processorSensor = processorSensor.get();
  }

  @Override
  public void armSetVoltage(double volts) {
    this.armVolts = volts;
  }

  @Override
  public void intakeSetVoltage(double volts) {
    this.intakeVolts = volts;
  }
}
