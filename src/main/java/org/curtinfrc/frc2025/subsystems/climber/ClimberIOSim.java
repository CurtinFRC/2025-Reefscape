package org.curtinfrc.frc2025.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIOSim implements ClimberIO {
  private DCMotorSim pivotMotorSim;
  private DCMotorSim grabberMotorSim;

  private DCMotor pivotMotor = DCMotor.getNEO(1);
  private DCMotor grabberMotor = DCMotor.getNEO(1);

  public ClimberIOSim() {
    pivotMotorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(pivotMotor, 0.025, 4.0), pivotMotor);
    grabberMotorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(grabberMotor, 0.025, 4.0), grabberMotor);
  }

  private double pivotAppliedVoltage = 0.0;
  private double grabberAppliedVoltage = 0.0;

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    pivotMotorSim.update(0.02);
    grabberMotorSim.update(0.02);
    inputs.pivotAppliedVoltage = pivotMotorSim.getInputVoltage();
    inputs.pivotCurrent = pivotMotorSim.getCurrentDrawAmps();
    inputs.pivotEncoderPosition = pivotMotorSim.getAngularPositionRad();

    inputs.grabberAppliedVoltage = grabberMotorSim.getInputVoltage();
    inputs.grabberCurrent = grabberMotorSim.getCurrentDrawAmps();
  }

  @Override
  public void setPivotVoltage(double voltage) {
    pivotAppliedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
    pivotMotorSim.setInputVoltage(pivotAppliedVoltage);
  }

  @Override
  public void setGrabberVoltage(double voltage) {
    grabberAppliedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
    grabberMotorSim.setInputVoltage(grabberAppliedVoltage);
  }

  @Override
  public void goToPivotSetpoint() {
    System.out.println("Going to pivot setpoint");
    pivotMotorSim.setAngle(
        ClimberConstants.pivotMotorTargetPositionRotations
            * (2 * Math.PI)); // convert rotations to radians
    System.out.println("Pivot setpoint: " + ClimberConstants.pivotMotorTargetPositionRotations);
  }

  @Override
  public boolean pivotIsStable() {
    return true;
  }
}
