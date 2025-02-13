package org.curtinfrc.frc2025.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import org.curtinfrc.frc2025.generated.TunerConstants;

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
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontRight.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));
  public static final double kT = 1; // TODO figure this out
}
