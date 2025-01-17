package org.curtinfrc.frc2025.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class ElevatorIONeo implements ElevatorIO {
  protected final SparkMax elevatorMotor =
      new SparkMax(ElevatorConstants.motorPort, MotorType.kBrushless);
  protected final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
  protected ElevatorConstants.Setpoints setpoint = ElevatorConstants.Setpoints.COLLECT;

  public ElevatorIONeo() {}

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.point = this.setpoint;
    inputs.encoderReading = elevatorEncoder.getPosition();
    inputs.motorCurrent = elevatorMotor.getOutputCurrent();
    inputs.motorTemp = elevatorMotor.getMotorTemperature();
    inputs.motorVelocity = elevatorEncoder.getVelocity();
  }
}
