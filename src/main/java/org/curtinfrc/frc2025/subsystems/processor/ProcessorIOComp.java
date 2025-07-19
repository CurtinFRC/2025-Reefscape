package org.curtinfrc.frc2025.subsystems.processor;

import static org.curtinfrc.frc2025.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import org.curtinfrc.frc2025.util.PhoenixUtil;

public class ProcessorIOComp implements ProcessorIO {
  private static final int ID = 20;
  private static final CurrentLimitsConfigs currentLimits =
      new CurrentLimitsConfigs().withSupplyCurrentLimit(20).withStatorCurrentLimit(40);

  private final TalonFX motor = new TalonFX(ID);

  private final DigitalInput processorSensor = new DigitalInput(999); // change later

  private final StatusSignal<Voltage> voltage = motor.getMotorVoltage();
  private final StatusSignal<Current> current = motor.getStatorCurrent();
  private final StatusSignal<Angle> position = motor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();

  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  public ProcessorIOComp() {
    tryUntilOk(
        5,
        () ->
            motor
                .getConfigurator()
                .apply(
                    new TalonFXConfiguration()
                        .withMotorOutput(
                            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
                        .withCurrentLimits(currentLimits)));
    BaseStatusSignal.setUpdateFrequencyForAll(20.0, velocity, voltage, current, position);
    motor.optimizeBusUtilization();

    // Register signals to be updated
    PhoenixUtil.registerSignals(false, velocity, voltage, current, position);
  }

  @Override
  public void updateInputs(ProcessorIOInputs inputs) {
    // motor.setControl(voltageRequest);
    // inputs.appliedVolts = voltage.getValueAsDouble();
    // inputs.currentAmps = current.getValueAsDouble();
    // inputs.positionRotations = position.getValueAsDouble();
    // inputs.angularVelocityRotationsPerMinute = velocity.getValueAsDouble();
    // inputs.frontSensor = frontSensor.get();
    // inputs.backSensor = backSensor.get();
  }

  @Override
  public void setVoltage(double volts) {
    voltageRequest.withOutput(volts);
  }
}