package org.curtinfrc.subsystems.intake;

import static org.curtinfrc.util.SparkUtil.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeIOSparkMax implements IntakeIO {
  private final SparkMax intakeMotor = new SparkMax(10, MotorType.kBrushless);

  public IntakeIOSparkMax() {
    var config = new SparkMaxConfig();
    config.voltageCompensation(12.0).smartCurrentLimit(80);

    tryUntilOk(
        intakeMotor,
        5,
        () ->
            intakeMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {}

  @Override
  public void intake() {
    intakeMotor.setVoltage(IntakeConstants.intakeVolts);
  }

  @Override
  public void stop() {
    intakeMotor.stopMotor();
  }
}
