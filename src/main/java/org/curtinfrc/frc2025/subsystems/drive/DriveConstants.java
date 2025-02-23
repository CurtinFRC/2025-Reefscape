package org.curtinfrc.frc2025.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import org.curtinfrc.frc2025.generated.CompTunerConstants;

public final class DriveConstants {
  public static final double DEADBAND = 0;
  public static final double ANGLE_KP = 5.0;
  public static final double ANGLE_KD = 0.4;
  public static final double ANGLE_MAX_VELOCITY = 8.0;
  public static final double ANGLE_MAX_ACCELERATION = 20.0;
  public static final double FF_START_DELAY = 2.0; // Secs
  public static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  public static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  public static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
  public static final double ODOMETRY_FREQUENCY =
      new CANBus(CompTunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(
                  CompTunerConstants.FrontLeft.LocationX, CompTunerConstants.FrontRight.LocationY),
              Math.hypot(
                  CompTunerConstants.FrontRight.LocationX,
                  CompTunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(
                  CompTunerConstants.BackLeft.LocationX, CompTunerConstants.BackLeft.LocationY),
              Math.hypot(
                  CompTunerConstants.BackRight.LocationX, CompTunerConstants.BackRight.LocationY)));
  public static final double kT =
      -DCMotor.getKrakenX60Foc(1).KtNMPerAmp
          * CompTunerConstants.kDriveGearRatio; // TODO figure this out
}
