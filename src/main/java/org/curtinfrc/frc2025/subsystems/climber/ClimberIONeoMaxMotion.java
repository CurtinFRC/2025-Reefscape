package org.curtinfrc.frc2025.subsystems.climber;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.curtinfrc.frc2025.util.SparkUtil;

public class ClimberIONeoMaxMotion extends ClimberIONeo {
  private SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();
  private SparkClosedLoopController grabberController = grabberMotor.getClosedLoopController();

  public ClimberIONeoMaxMotion() {
    var pivotConfig = new SparkMaxConfig();
    var grabberConfig = new SparkMaxConfig();

    pivotConfig.voltageCompensation(12.0).smartCurrentLimit(80);
    grabberConfig.voltageCompensation(12.0).smartCurrentLimit(80);

    pivotConfig
        .closedLoop
        .p(ClimberConstants.pivotkP)
        .i(ClimberConstants.pivotkI)
        .d(ClimberConstants.pivotkD)
        .outputRange(ClimberConstants.pivotkMinOutput, ClimberConstants.pivotkMaxOutput);

    grabberConfig
        .closedLoop
        .p(ClimberConstants.grabberkP)
        .i(ClimberConstants.grabberkI)
        .d(ClimberConstants.grabberkD)
        .outputRange(ClimberConstants.grabberkMinOutput, ClimberConstants.grabberkMaxOutput);

    pivotConfig.closedLoop.velocityFF(1 / ClimberConstants.pivotkV);
    grabberConfig.closedLoop.velocityFF(1 / ClimberConstants.grabberkV);

    pivotConfig
        .closedLoop
        .maxMotion
        .maxVelocity(ClimberConstants.pivotMaxVelocity)
        .maxAcceleration(ClimberConstants.pivotMaxAcceleration)
        .allowedClosedLoopError(ClimberConstants.pivotAllowedError);

    grabberConfig
        .closedLoop
        .maxMotion
        .maxVelocity(ClimberConstants.grabberMaxVelocity)
        .maxAcceleration(ClimberConstants.grabberMaxAcceleration)
        .allowedClosedLoopError(ClimberConstants.grabberAllowedError);

    SparkUtil.tryUntilOk(
        pivotMotor,
        5,
        () ->
            pivotMotor.configure(
                pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkUtil.tryUntilOk(
        grabberMotor,
        5,
        () ->
            grabberMotor.configure(
                grabberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void goToPivotSetpoint() {
    pivotController.setReference(
        ClimberConstants.pivotMotorTargetPositionRotations, ControlType.kPosition);
  }

  @Override
  public void goToGrabberSetpoint() {
    grabberController.setReference(
        ClimberConstants.grabberMotorTargetPositionRotations, ControlType.kPosition);
  }

  @Override
  public boolean grabberIsStable() {
    double position = grabberEncoder.getPosition(); // rotations
    return ClimberConstants.grabberMotorTargetPositionRotations - ClimberConstants.grabberTolerance
            < position
        && position
            < ClimberConstants.grabberMotorTargetPositionRotations
                + ClimberConstants.grabberTolerance;
  }
}
