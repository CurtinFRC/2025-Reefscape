package org.curtinfrc.frc2025.subsystems.ejector;

import static org.curtinfrc.frc2025.subsystems.ejector.EjectorConstants.*;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.curtinfrc.frc2025.util.TestUtil;

public class EjectorIOSim implements EjectorIO {
  private DCMotor intakeMotor = DCMotor.getNEO(1);
  private DCMotorSim intakeMotorSim;
  private SimDevice sensorImplBack;
  private SimBoolean frontSensor;
  private SimDevice sensorImplFront;
  private SimBoolean backSensor;
  private double volts = 0;

  public EjectorIOSim(TestUtil tests) {
    intakeMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(intakeMotor, ejectorMoi, ejectorReduction),
            intakeMotor);

    sensorImplFront = SimDevice.create("EjectorFront", frontSensorPort);
    frontSensor = sensorImplFront.createBoolean("IsTriggered", Direction.kInput, false);

    sensorImplBack = SimDevice.create("EjectorBack", backSensorPort);
    backSensor = sensorImplBack.createBoolean("IsTriggered", Direction.kInput, false);

    tests.addInput(tests.new DigitalSensor(frontSensor, "EjectorFront"));
    tests.addInput(tests.new DigitalSensor(backSensor, "EjectorBack"));
  }

  @Override
  public void updateInputs(EjectorIOInputs inputs) {
    intakeMotorSim.setInputVoltage(volts);
    intakeMotorSim.update(0.02);
    inputs.appliedVolts = intakeMotorSim.getInputVoltage();
    inputs.currentAmps = intakeMotorSim.getCurrentDrawAmps();
    inputs.positionRotations = intakeMotorSim.getAngularPositionRotations();
    inputs.angularVelocityRotationsPerMinute = intakeMotorSim.getAngularVelocityRPM();
    inputs.frontSensor = frontSensor.get();
  }

  @Override
  public void setVoltage(double volts) {
    this.volts = volts;
    intakeMotorSim.setInputVoltage(volts);
  }
}
