package org.curtinfrc.frc2025.subsystems.climber;

// import com.revrobotics.RelativeEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;

public class ClimberIONeo implements ClimberIO {
  protected final SparkMax grabberMotor =
      new SparkMax(ClimberConstants.grabberMotorPort, MotorType.kBrushless);
  protected final RelativeEncoder grabberEncoder = grabberMotor.getEncoder();

  public ClimberIONeo() {}

  private double grabberAppliedVoltage = 0.0;

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.grabberAppliedVoltage = grabberMotor.getBusVoltage() * grabberMotor.getAppliedOutput();
    inputs.grabberCurrent = grabberMotor.getOutputCurrent();
    inputs.grabberEncoderPosition = grabberEncoder.getPosition();

    inputs.grabberIsStable = grabberIsStable();
  }

  @Override
  public void setGrabberVoltage(double voltage) {
    grabberAppliedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
    grabberMotor.set(grabberAppliedVoltage);
  }
}
