package org.curtinfrc.frc2025.subsystems.ejector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class EjectorIOSim implements EjectorIO {
  private DCMotor ejectorMotor = DCMotor.getNEO(1);
  private DCMotorSim ejectorMotorSim;

  public EjectorIOSim() {
    ejectorMotorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(ejectorMotor, 0.025, 4.0), ejectorMotor);
  }

  @Override
  public void updateInputs(EjectorIOInputs inputs) {
    ejectorMotorSim.update(0.02);
    inputs.ejectorEncoderOutput = ejectorMotorSim.getAngularVelocityRadPerSec();
  }

  @Override
  public void achieveEjectorRPM() {
    ejectorMotorSim.setAngularVelocity(EjectorConstants.goalRPM);
  }

  @Override
  public boolean ejectorAtRPM() {
    double angularVelocity = ejectorMotorSim.getAngularVelocityRadPerSec();
    return EjectorConstants.goalRPM - EjectorConstants.ejectorTolerance < angularVelocity
        && angularVelocity < EjectorConstants.goalRPM + EjectorConstants.ejectorTolerance;
  }
}
