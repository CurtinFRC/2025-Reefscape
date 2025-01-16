package org.curtinfrc.frc2025.subsystems.climber;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.curtinfrc.frc2025.util.SparkUtil;

public class ClimberIONeoMaxMotion extends ClimberIONeo {
  private SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();

  public ClimberIONeoMaxMotion() {
    var config = new SparkMaxConfig();
    config.voltageCompensation(12.0).smartCurrentLimit(80);

    config
        .closedLoop
        .p(ClimberConstants.pivotkP)
        .i(ClimberConstants.pivotkI)
        .d(ClimberConstants.pivotkD)
        .outputRange(ClimberConstants.pivotkMinOutput, ClimberConstants.pivotkMaxOutput);

    config.closedLoop.velocityFF(1 / ClimberConstants.pivotkV);

    config
        .closedLoop
        .maxMotion
        .maxVelocity(ClimberConstants.pivotMaxVelocity)
        .maxAcceleration(ClimberConstants.pivotMaxAcceleration)
        .allowedClosedLoopError(ClimberConstants.pivotAllowedError);

    SparkUtil.tryUntilOk(
        pivotMotor,
        5,
        () ->
            pivotMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void goToPivotSetpoint() {
    pivotController.setReference(
        ClimberConstants.pivotMotorTargetPositionRotations, ControlType.kPosition);
  }

  @Override
  public boolean pivotIsStable() {
    double position = pivotEncoder.getPosition();
    return ClimberConstants.pivotMotorTargetPositionRotations - ClimberConstants.pivotTolerance
            < position
        && position
            < ClimberConstants.pivotMotorTargetPositionRotations + ClimberConstants.pivotTolerance;
  }
}
