package org.curtinfrc.frc2025.subsystems.climber;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.curtinfrc.frc2025.util.SparkUtil;

public class ClimberIONeoMaxMotion extends ClimberIONeo {
  private SparkClosedLoopController grabberController = grabberMotor.getClosedLoopController();

  public ClimberIONeoMaxMotion() {
    var grabberConfig = new SparkMaxConfig();

    grabberConfig.voltageCompensation(12.0).smartCurrentLimit(80);

    grabberConfig
        .closedLoop
        .p(ClimberConstants.grabberkP)
        .i(ClimberConstants.grabberkI)
        .d(ClimberConstants.grabberkD)
        .outputRange(ClimberConstants.grabberkMinOutput, ClimberConstants.grabberkMaxOutput);

    grabberConfig.closedLoop.velocityFF(1 / ClimberConstants.grabberkV);

    grabberConfig
        .closedLoop
        .maxMotion
        .maxVelocity(ClimberConstants.grabberMaxVelocity)
        .maxAcceleration(ClimberConstants.grabberMaxAcceleration)
        .allowedClosedLoopError(ClimberConstants.grabberAllowedError);

    SparkUtil.tryUntilOk(
        grabberMotor,
        5,
        () ->
            grabberMotor.configure(
                grabberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void goToGrabberSetpoint() {
    grabberController.setReference(
        ClimberConstants.grabberMotorTargetPositionRotations, ControlType.kPosition);
  }

  @Override
  public boolean grabberIsStable() {
    return false; // TODO: Work out how to implement grabberIsStable
  }
}
