package org.curtinfrc.frc2025.subsystems.popper;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.curtinfrc.frc2025.util.SparkUtil;

public class PopperIOComp implements PopperIO {
  private static final int ID = 43;
  private static final int CURRENT_LIMIT = 15;

  private final SparkMax motor = new SparkMax(ID, MotorType.kBrushless);

  public PopperIOComp() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(0, CURRENT_LIMIT).idleMode(IdleMode.kCoast).inverted(true);

    SparkUtil.tryUntilOk(
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(PopperIOInputs inputs) {
    inputs.appliedVolts = motor.getBusVoltage() * motor.getAppliedOutput();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.positionRotations = motor.getEncoder().getPosition();
    inputs.angularVelocityRotationsPerMinute = motor.getEncoder().getVelocity();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
