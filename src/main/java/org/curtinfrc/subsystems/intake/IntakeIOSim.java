package org.curtinfrc.subsystems.intake;

import static org.curtinfrc.subsystems.intake.IntakeConstants.intakeBackSensorPort;
import static org.curtinfrc.subsystems.intake.IntakeConstants.intakeFrontSensorPort;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private DCMotor intakeMotor = DCMotor.getNEO(1);
  private DCMotorSim intakeMotorSim;
  private SimDevice frontImpl;
  private SimBoolean frontSensor;
  private SimDevice backImpl;
  private SimBoolean backSensor;
  private double volts = 0;

  public IntakeIOSim() {
    intakeMotorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(intakeMotor, 0.025, 4.0), intakeMotor);

    frontImpl = SimDevice.create("IntakeSensorFront", intakeFrontSensorPort);
    frontSensor = frontImpl.createBoolean("IsTriggered", Direction.kInput, false);
    backImpl = SimDevice.create("IntakeSensorBack", intakeBackSensorPort);
    backSensor = backImpl.createBoolean("IsTriggered", Direction.kInput, false);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeMotorSim.setInputVoltage(volts);
    intakeMotorSim.update(0.02);
    inputs.appliedVolts = intakeMotorSim.getInputVoltage();
    inputs.currentAmps = intakeMotorSim.getCurrentDrawAmps();
    inputs.positionRotations = intakeMotorSim.getAngularPositionRotations();
    inputs.angularVelocityRotationsPerMinute = intakeMotorSim.getAngularVelocityRPM();
    inputs.frontSensor = frontSensor.get();
    inputs.backSensor = backSensor.get();
  }

  @Override
  public void setVoltage(double volts) {
    this.volts = volts;
    intakeMotorSim.setInputVoltage(volts);
  }
}
