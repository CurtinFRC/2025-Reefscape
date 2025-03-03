package org.curtinfrc.frc2025.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIOSim implements ClimberIO {
  private DCMotorSim climberSim;
  private DCMotor grabberMotor = DCMotor.getNEO(1);

  public ClimberIOSim() {
    climberSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(grabberMotor, 0.025, 4.0), grabberMotor);
  }

  private double voltage = 0.0;

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    climberSim.update(0.02);

    inputs.appliedVoltage = voltage;
    inputs.currentAmps = climberSim.getCurrentDrawAmps();
    inputs.positionRotations = climberSim.getAngularPositionRad();
    inputs.angularVelocityRotationsPerMinute = climberSim.getAngularVelocityRPM();
  }

  @Override
  public void setVoltage(double voltage) {
    this.voltage = MathUtil.clamp(voltage, -12.0, 12.0);
    climberSim.setInputVoltage(voltage);
  }
}
