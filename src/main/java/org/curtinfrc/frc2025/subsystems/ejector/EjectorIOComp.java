package org.curtinfrc.frc2025.subsystems.ejector;

import static org.curtinfrc.frc2025.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

public class EjectorIOComp implements EjectorIO {
  private static final int ID = 53;
  private static final int FOLLOWER_ID = 46;
  private static final CurrentLimitsConfigs currentLimits =
      new CurrentLimitsConfigs().withSupplyCurrentLimit(20).withStatorCurrentLimit(40);
  private static final TalonFXConfiguration leaderConfig =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive))
          .withSlot0(new Slot0Configs().withKP(0.5).withKV(0.33))
          .withCurrentLimits(currentLimits);
  private static final TalonFXConfiguration followerConfig =
      new TalonFXConfiguration()
          .withCurrentLimits(currentLimits)
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

  private final TalonFX motor = new TalonFX(ID);
  private final TalonFX follower = new TalonFX(FOLLOWER_ID);

  private final DigitalInput frontSensor = new DigitalInput(4);
  private final DigitalInput backSensor = new DigitalInput(2);

  private final StatusSignal<Voltage> voltage = motor.getMotorVoltage();
  private final StatusSignal<Current> current = motor.getStatorCurrent();
  private final StatusSignal<Angle> position = motor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();

  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(true);
  private final Follower followRequest = new Follower(ID, true);

  public EjectorIOComp() {
    tryUntilOk(5, () -> motor.getConfigurator().apply(leaderConfig));
    tryUntilOk(5, () -> follower.getConfigurator().apply(followerConfig));
    BaseStatusSignal.setUpdateFrequencyForAll(20.0, velocity, voltage, current, position);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(EjectorIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocity, voltage, current, position);
    follower.setControl(followRequest);
    inputs.appliedVolts = voltage.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.positionRotations = position.getValueAsDouble();
    inputs.angularVelocityRotationsPerSecond = velocity.getValueAsDouble();
    inputs.frontSensor = frontSensor.get();
    inputs.backSensor = backSensor.get();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setVelocity(double speedRotationsPerSecond) {
    motor.setControl(velocityRequest.withVelocity(speedRotationsPerSecond));
  }
}
