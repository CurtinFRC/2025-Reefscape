package org.curtinfrc.frc2025.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
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
  public static final double kT =
      -DCMotor.getKrakenX60Foc(1).KtNMPerAmp
          * TunerConstants.kDriveGearRatio; // TODO figure this out

  // TODO
  public static enum DriveSetpoints {
    A(new Pose2d(3.75, 5.00, Rotation2d.fromDegrees(300))),
    B(new Pose2d(4.03, 5.15, Rotation2d.fromDegrees(300))),
    C(Pose2d.kZero),
    D(Pose2d.kZero),
    E(Pose2d.kZero),
    F(Pose2d.kZero),
    G(Pose2d.kZero),
    H(Pose2d.kZero),
    I(new Pose2d(4.00, 2.86, Rotation2d.fromDegrees(60))),
    J(new Pose2d(3.72, 3.02, Rotation2d.fromDegrees(60))),
    K(new Pose2d(3.27, 3.85, Rotation2d.kZero)),
    L(new Pose2d(3.27, 4.18, Rotation2d.kZero)),
    PROCESSOR(Pose2d.kZero),
    LEFT_HP(new Pose2d(1.18, 7.24, Rotation2d.fromDegrees(125.989 + 180))),
    RIGHT_HP(new Pose2d(1.10, 0.89, Rotation2d.fromDegrees(125.989 + 180).unaryMinus())),
    NULL(Pose2d.kZero);

    public final Pose2d pose;

    DriveSetpoints(Pose2d pose) {
      this.pose = pose;
    }
  }
}
