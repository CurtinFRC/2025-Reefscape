package org.curtinfrc.frc2025.subsystems.drive;

import static org.curtinfrc.frc2025.subsystems.drive.DriveConstants.ODOMETRY_FREQUENCY;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Queue;
import java.util.function.Supplier;
import org.curtinfrc.frc2025.generated.TunerConstants;
import org.curtinfrc.frc2025.util.DeltaTime;

/** Simulated implementation of GyroIO for testing purposes. */
public class GyroIOSim implements GyroIO {
  private final Pigeon2 pigeon =
      new Pigeon2(
          TunerConstants.DrivetrainConstants.Pigeon2Id,
          TunerConstants.DrivetrainConstants.CANBusName);

  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Pigeon2SimState pigeonSimState;

  private final Supplier<SwerveDriveKinematics> kinematics;

  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  private Rotation2d rawGyroYaw = Rotation2d.fromDegrees(0);
  private final DeltaTime deltaTime;
  private final Supplier<SwerveModuleState[]> getStates;

  public GyroIOSim(
      Supplier<SwerveDriveKinematics> kinematics, Supplier<SwerveModuleState[]> getStates) {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    pigeonSimState = pigeon.getSimState();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getYaw());

    this.getStates = getStates;
    this.deltaTime = new DeltaTime(true);
    this.kinematics = kinematics;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    final SwerveModuleState[] moduleStates = getStates.get();

    rawGyroYaw =
        rawGyroYaw.plus(
            Rotation2d.fromRadians(
                kinematics.get().toChassisSpeeds(moduleStates).omegaRadiansPerSecond
                    * deltaTime.get()));

    pigeonSimState.setRawYaw(rawGyroYaw.getDegrees());

    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
