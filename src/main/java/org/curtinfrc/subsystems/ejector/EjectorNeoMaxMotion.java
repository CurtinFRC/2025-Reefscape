package org.curtinfrc.subsystems.ejector;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class EjectorNeoMaxMotion extends EjectorIONEO {
  private SparkClosedLoopController ejectorController = ejectorNeo.getClosedLoopController();

  public EjectorNeoMaxMotion() {
    var config = new SparkMaxConfig();
    config
        .smartCurrentLimit(EjectorConstants.ejectorCurrentLimit)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12.0);

    config.closedLoop.p(EjectorConstants.kP).i(EjectorConstants.kI).d(EjectorConstants.kD);

    config.closedLoop.velocityFF(1 / EjectorConstants.kV);

    config
        .closedLoop
        .maxMotion
        .maxVelocity(EjectorConstants.ejectorMaxVelocity)
        .allowedClosedLoopError(0.01);
  }

  @Override
  public void achieveEjectorRPM() {
    ejectorController.setReference(EjectorConstants.goalRPM, ControlType.kPosition);
  }
  // @Override
  // public void achieveEjectorRPM(){
  //   ejectorController.setReference(EjectorConstants.goalRPM, ControlType.kPosition);
  // }

  @Override
  public boolean ejectorAtRPM() {
    double angularVelocity = ejectorEncoder.getVelocity();
    return EjectorConstants.goalRPM - EjectorConstants.ejectorTolerance < angularVelocity
        && angularVelocity < EjectorConstants.goalRPM + EjectorConstants.ejectorTolerance;
  }
}
