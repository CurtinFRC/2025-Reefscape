package org.curtinfrc.frc2025.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class ElevatorIONeo implements ElevatorIO {
  protected final SparkMax elevatorMotor = new SparkMax(10, MotorType.kBrushless);
  protected final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
  protected ElevatorConstants.Setpoints setpoint = ElevatorConstants.Setpoints.NONE;

  public ElevatorIONeo() {}

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.point = this.setpoint;
    inputs.encoderReading = elevatorEncoder.getPosition();
  }
}
