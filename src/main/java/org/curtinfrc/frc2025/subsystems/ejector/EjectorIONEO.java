package org.curtinfrc.frc2025.subsystems.ejector;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
// import org.curtinfrc.subsystems.Ejector.EjectorIO.EjectorIOInputs;

public class EjectorIONEO implements EjectorIO {
  protected final SparkMax ejectorNeo = new SparkMax(99, MotorType.kBrushless);
  protected final RelativeEncoder ejectorEncoder = ejectorNeo.getEncoder();

  public EjectorIONEO() {}

  @Override
  public void updateInputs(EjectorIOInputs inputs) {
    inputs.ejectorEncoderOutput = ejectorEncoder.getVelocity();
    // inputs.encoderOutput = intakeEncoder.getVelocity();
    inputs.appliedVolts = ejectorNeo.getBusVoltage() * ejectorNeo.getAppliedOutput();
  }
}
