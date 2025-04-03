package org.curtinfrc.frc2025.subsystems.elevator;

import static org.curtinfrc.frc2025.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import org.curtinfrc.frc2025.subsystems.elevator.ElevatorConstants.ElevatorSetpoints;

public class ElevatorIOComp implements ElevatorIO {
  private static final double pulleyRadiusMeters = 0.03055;
  private static final double gearing = 8.1818;
  private static final int ID = 7;
  private static final int FOLLOWER_ID = 40;
  private static final TalonFXConfiguration config =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withStatorCurrentLimit(60));

  private final TalonFX motor = new TalonFX(ID);
  private final TalonFX follower = new TalonFX(FOLLOWER_ID);

  private DigitalInput reset = new DigitalInput(0);

  private final StatusSignal<Voltage> voltage = motor.getMotorVoltage();
  private final StatusSignal<Current> current = motor.getStatorCurrent();
  private final StatusSignal<Angle> position = motor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();

  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final Follower followRequest = new Follower(ID, false);

  public ElevatorIOComp() {
    BaseStatusSignal.setUpdateFrequencyForAll(20.0, velocity, voltage, current, position);
    motor.optimizeBusUtilization();

    var slot0Configs = config.Slot0;
    slot0Configs.kS = 0;
    slot0Configs.kV = ElevatorConstants.kV;
    slot0Configs.kA = ElevatorConstants.kA;
    slot0Configs.kP = ElevatorConstants.kP;
    slot0Configs.kI = 0;
    slot0Configs.kD = ElevatorConstants.kD;
    slot0Configs.kG = ElevatorConstants.kG;

    var motionMagicConfigs = config.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = positionMetresToRotations(3.54);
    motionMagicConfigs.MotionMagicAcceleration = positionMetresToRotations(55.7);
    motionMagicConfigs.MotionMagicJerk = 1;

    tryUntilOk(5, () -> motor.getConfigurator().apply(config));
    tryUntilOk(5, () -> follower.getConfigurator().apply(config));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocity, voltage, current, position);
    motor.setControl(voltageRequest);
    follower.setControl(followRequest);
    inputs.appliedVolts = voltage.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.positionRotations = position.getValueAsDouble();
    inputs.angularVelocityRotationsPerMinute = velocity.getValueAsDouble();
    inputs.hominSensor = reset.get();
  }

  @Override
  public void setVoltage(double volts) {
    voltageRequest.withOutput(volts);
  }

  @Override
  public void zero() {
    motor.setPosition(0);
  }

  @Override
  public double positionRotationsToMetres(double rotations) {
    return rotations * Math.PI * 2 * pulleyRadiusMeters / gearing;
  }

  @Override
  public double positionMetresToRotations(double metres) {
    return metres / (Math.PI * 2 * pulleyRadiusMeters) * gearing;
  }

  @Override
  public void goToSetpoint(ElevatorSetpoints point) {
    final MotionMagicVoltage req = new MotionMagicVoltage(0);
    motor.setControl(req.withPosition(point.setpoint));
  }
}
