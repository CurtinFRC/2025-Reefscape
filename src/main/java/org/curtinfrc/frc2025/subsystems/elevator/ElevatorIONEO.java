package org.curtinfrc.frc2025.subsystems.elevator;

import static org.curtinfrc.frc2025.subsystems.ejector.EjectorConstants.currentLimit;
import static org.curtinfrc.frc2025.subsystems.elevator.ElevatorConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import org.curtinfrc.frc2025.util.SparkUtil;

public class ElevatorIONEO implements ElevatorIO {
  protected final SparkMax elevatorMotor =
      new SparkMax(ElevatorConstants.motorPort, MotorType.kBrushless);
  protected final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
  protected DigitalInput touch = new DigitalInput(0);

  public ElevatorIONEO() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(0, currentLimit).idleMode(IdleMode.kCoast).inverted(true);

    SparkUtil.tryUntilOk(
        5,
        () ->
            elevatorMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.appliedVolts = elevatorMotor.getBusVoltage() * elevatorMotor.getAppliedOutput();
    inputs.currentAmps = elevatorMotor.getOutputCurrent();
    inputs.positionRotations = elevatorMotor.getEncoder().getPosition();
    inputs.angularVelocityRotationsPerMinute = elevatorMotor.getEncoder().getVelocity();
    inputs.touchSensor = touch.get();
  }

  @Override
  public void setVoltage(double volts) {
    elevatorMotor.setVoltage(volts);
  }

  @Override
  public void zero() {
    elevatorEncoder.setPosition(0);
  }
}
