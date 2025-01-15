package org.curtinfrc.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class IntakeIOSparkMax implements IntakeIO {
  private final SparkMax intakeNeo = new SparkMax(99, MotorType.kBrushless);
  private final RelativeEncoder intakeEncoder = intakeNeo.getEncoder();

  public IntakeIOSparkMax() {}

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // inputs.appliedVolts = appliedVolts.getValueAsDouble();
  }
}
