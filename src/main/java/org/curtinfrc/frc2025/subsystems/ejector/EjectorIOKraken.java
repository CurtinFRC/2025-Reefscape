package org.curtinfrc.frc2025.subsystems.ejector;

import static org.curtinfrc.frc2025.subsystems.ejector.EjectorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

public class EjectorIOKraken implements EjectorIO {
  private static final int ID = 45;

  private final TalonFX motor = new TalonFX(ID);
  private final DigitalInput frontSensor = new DigitalInput(frontSensorPort);
  private final DigitalInput backSensor = new DigitalInput(backSensorPort);

  private final StatusSignal<Voltage> voltage = motor.getMotorVoltage();
  private final StatusSignal<Current> current = motor.getStatorCurrent();
  private final StatusSignal<Angle> position = motor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();

  public EjectorIOKraken() {
    BaseStatusSignal.setUpdateFrequencyForAll(20.0, velocity, voltage, current, position);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(EjectorIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocity, voltage, current, position);
    inputs.appliedVolts = voltage.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.positionRotations = position.getValueAsDouble();
    inputs.angularVelocityRotationsPerMinute = position.getValueAsDouble();
    inputs.frontSensor = frontSensor.get();
    inputs.backSensor = backSensor.get();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
