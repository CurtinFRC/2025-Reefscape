package org.curtinfrc.frc2025.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import org.curtinfrc.frc2025.util.SparkUtil;
import org.curtinfrc.frc2025.util.TestUtil;

public class ClimberIOComp implements ClimberIO {
  private final SparkMax climberMotor = new SparkMax(9, MotorType.kBrushless);
  private final RelativeEncoder climberEncoder = climberMotor.getEncoder();
  private final Servo servo = new Servo(0);

  public ClimberIOComp(TestUtil tests) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(0, 60).idleMode(IdleMode.kBrake).inverted(false);

    SparkUtil.tryUntilOk(
        5,
        () ->
            climberMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tests.addInput(tests.new Motor(climberMotor, "ClimberNeo"));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.appliedVoltage = climberMotor.getBusVoltage() * climberMotor.getAppliedOutput();
    inputs.currentAmps = climberMotor.getOutputCurrent();
    inputs.positionRotations = climberEncoder.getPosition();
    inputs.angularVelocityRotationsPerMinute = climberEncoder.getVelocity();
    inputs.ratchet = servo.get();
  }

  @Override
  public void setVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -12.0, 12.0);
    climberMotor.set(voltage);
  }

  @Override
  public void engageRatchet() {
    servo.set(0.5);
  }

  @Override
  public void disableRatchet() {
    servo.set(0);
  }
}
