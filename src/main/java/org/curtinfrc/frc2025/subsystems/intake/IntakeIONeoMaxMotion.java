package org.curtinfrc.frc2025.subsystems.intake;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.curtinfrc.frc2025.util.SparkUtil;

public class IntakeIONeoMaxMotion extends IntakeIONEO {
  private SparkClosedLoopController intakeController = intakeNeo.getClosedLoopController();

  public IntakeIONeoMaxMotion() {
    var config = new SparkMaxConfig();
    config
        .smartCurrentLimit(IntakeConstants.intakeCurrentLimit)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12.0);

    // config
    //     .encoder
    //     .positionConversionFactor(2.0 * Math.PI / IntakeConstants.motorReduction)
    //     .velocityConversionFactor((2.0 * Math.PI) / 60.0 / IntakeConstants.motorReduction)
    //     .uvwMeasurementPeriod(10)
    //     .uvwAverageDepth(2);

    config.closedLoop.p(IntakeConstants.kP).i(IntakeConstants.kI).d(IntakeConstants.kD);

    config.closedLoop.velocityFF(1 / IntakeConstants.kV);

    config
        .closedLoop
        .maxMotion
        .maxVelocity(IntakeConstants.intakeMaxVelocity)
        .allowedClosedLoopError(IntakeConstants.intakeTolerance);

    SparkUtil.tryUntilOk(
        intakeNeo,
        5,
        () ->
            intakeNeo.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void achieveRPM() {
    intakeController.setReference(IntakeConstants.goalRPM, ControlType.kPosition);
  }

  @Override
  public boolean intakeAtRPM() {
    double angularVelocity = intakeEncoder.getVelocity();
    return IntakeConstants.goalRPM - IntakeConstants.intakeTolerance < angularVelocity
        && angularVelocity < IntakeConstants.goalRPM + IntakeConstants.intakeTolerance;
  }
}
