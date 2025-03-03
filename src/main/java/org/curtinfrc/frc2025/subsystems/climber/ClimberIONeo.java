package org.curtinfrc.frc2025.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import org.curtinfrc.frc2025.util.SparkUtil;

public class ClimberIONeo implements ClimberIO {
  private final SparkMax climberMotor =
      new SparkMax(ClimberConstants.grabberMotorPort, MotorType.kBrushless);
  protected final RelativeEncoder climberEncoder = climberMotor.getEncoder();

  public ClimberIONeo() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .smartCurrentLimit(0, ClimberConstants.currentLimit)
        .idleMode(IdleMode.kBrake)
        .inverted(false); // TODO: test direction

    SparkUtil.tryUntilOk(
        5,
        () ->
            climberMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.appliedVoltage = climberMotor.getBusVoltage() * climberMotor.getAppliedOutput();
    inputs.currentAmps = climberMotor.getOutputCurrent();
    inputs.positionRotations = climberEncoder.getPosition();
    inputs.angularVelocityRotationsPerMinute = climberEncoder.getVelocity();
  }

  @Override
  public void setVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -12.0, 12.0);
    climberMotor.set(voltage);
  }
}
