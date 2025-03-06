package org.curtinfrc.frc2025.subsystems.intake;

import static org.curtinfrc.frc2025.subsystems.intake.IntakeConstants.intakeBackSensorPort;
import static org.curtinfrc.frc2025.subsystems.intake.IntakeConstants.intakeFrontSensorPort;
import static org.curtinfrc.frc2025.subsystems.intake.IntakeConstants.intakeMoi;
import static org.curtinfrc.frc2025.subsystems.intake.IntakeConstants.motorReduction;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private final DCMotor intakeMotor = DCMotor.getNEO(1);
  private final DCMotorSim intakeMotorSim;
  private final SimDevice frontImpl;
  private final SimBoolean frontSensor;
  private final SimDevice backImpl;
  private final SimBoolean backSensor;
  private double volts = 0;

  public IntakeIOSim() {
    intakeMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(intakeMotor, intakeMoi, motorReduction),
            intakeMotor);

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
  }
}
