package org.curtinfrc.frc2025.subsystems.climber;

// import com.revrobotics.RelativeEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class ClimberIONeo implements ClimberIO {
  protected final SparkMax pivotMotor =
      new SparkMax(ClimberConstants.pivotMotorPort, MotorType.kBrushless);
  protected final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

  protected final SparkMax grabberMotor =
      new SparkMax(ClimberConstants.grabberMotorPort, MotorType.kBrushless);
  protected final RelativeEncoder grabberEncoder = grabberMotor.getEncoder();

  public ClimberIONeo() {}

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.grabberAppliedVoltage = grabberMotor.getBusVoltage() * grabberMotor.getAppliedOutput();
    inputs.grabberCurrent = grabberMotor.getOutputCurrent();
    inputs.grabberEncoderPosition = grabberEncoder.getPosition();

    inputs.pivotAppliedVoltage = pivotMotor.getBusVoltage() * pivotMotor.getAppliedOutput();
    inputs.pivotCurrent = pivotMotor.getOutputCurrent();
    inputs.pivotEncoderPosition = pivotEncoder.getPosition();
  }

  @Override
  public void setPivotVoltage(double voltage) {
    pivotMotor.set(voltage);
  }

  @Override
  public void setGrabberVoltage(double voltage) {
    grabberMotor.set(voltage);
  }
}