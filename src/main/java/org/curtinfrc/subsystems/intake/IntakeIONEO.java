package org.curtinfrc.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class IntakeIONEO implements IntakeIO {
  private final SparkMax intakeNeo = new SparkMax(99, MotorType.kBrushless);
  private final RelativeEncoder intakeEncoder = intakeNeo.getEncoder();

  public IntakeIONEO() {}

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.encoderOutput = intakeEncoder.getVelocity();
    // inputs.encoderOutput = intakeEncoder.getVelocity();
    inputs.appliedVolts = intakeNeo.getBusVoltage() * intakeNeo.getAppliedOutput();
  }
}
