package org.curtinfrc.frc2025.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIOSim implements ClimberIO {
  private DCMotorSim grabberMotorSim;
  private DCMotor grabberMotor = DCMotor.getNEO(1);

  public ClimberIOSim() {
    grabberMotorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(grabberMotor, 0.025, 4.0), grabberMotor);
  }

  private double grabberAppliedVoltage = 0.0;

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    grabberMotorSim.update(0.02);

    inputs.grabberAppliedVoltage = grabberMotorSim.getInputVoltage();
    inputs.grabberCurrent = grabberMotorSim.getCurrentDrawAmps();
    inputs.grabberEncoderPosition = grabberMotorSim.getAngularPositionRad();

    inputs.grabberIsStable = grabberIsStable();
  }

  @Override
  public void setGrabberVoltage(double voltage) {
    grabberAppliedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
    grabberMotorSim.setInputVoltage(grabberAppliedVoltage);
  }

  @Override
  public void goToGrabberSetpoint() {
    grabberMotorSim.setAngle(
        ClimberConstants.grabberMotorTargetPositionRotations
            * (2 * Math.PI)); // convert rotations to radians
  }

  @Override
  public boolean grabberIsStable() {
    return true;
  }
}
