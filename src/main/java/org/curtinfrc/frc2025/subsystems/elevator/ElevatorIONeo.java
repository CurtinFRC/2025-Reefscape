package org.curtinfrc.frc2025.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class ElevatorIONeo implements ElevatorIO {
  protected final SparkMax elevatorMotor = new SparkMax(10, MotorType.kBrushless);
  protected final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

  public ElevatorIONeo() {}

  @Override
  public void applyVoltage(double volts) {}

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {}
}
