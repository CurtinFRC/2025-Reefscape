package org.curtinfrc.frc2025.subsystems.drive;

import static org.curtinfrc.frc2025.subsystems.vision.VisionConstants.aprilTagLayout;

import choreo.util.ChoreoAllianceFlipUtil;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.curtinfrc.frc2025.Constants;
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

  // TODO
  public static enum DriveSetpoints implements StructSerializable {
    A(aprilTagLayout.getTagPose(18).get(), true),
    B(aprilTagLayout.getTagPose(18).get(), false),
    C(aprilTagLayout.getTagPose(17).get(), true),
    D(aprilTagLayout.getTagPose(17).get(), false),
    E(aprilTagLayout.getTagPose(22).get(), true),
    F(aprilTagLayout.getTagPose(22).get(), false),
    G(aprilTagLayout.getTagPose(21).get(), true),
    H(aprilTagLayout.getTagPose(21).get(), false),
    I(aprilTagLayout.getTagPose(20).get(), true),
    J(aprilTagLayout.getTagPose(20).get(), false),
    K(aprilTagLayout.getTagPose(19).get(), true),
    L(aprilTagLayout.getTagPose(19).get(), false),
    // PROCESSOR(Pose2d.kZero),
    LEFT_HP(new Pose2d(1.313582181930542 , 7.24, Rotation2d.fromDegrees(125.989 + 180))),
    RIGHT_HP(new Pose2d(1.313582181930542 , 0.89, Rotation2d.fromDegrees(125.989 + 180).unaryMinus()));

    private final Pose2d pose;

    public Pose2d getPose() {
      boolean isFlipped =
          DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
      if (isFlipped) {
        return ChoreoAllianceFlipUtil.flip(pose);
      }
      return pose;
    }

    static Pose3d mapPose(Pose3d pose) {
      double angle = pose.getRotation().getAngle();
      return new Pose3d(
          pose.getX() + Math.cos(angle) * Constants.ROBOT_X / 2000.0,
          pose.getY() + Math.sin(angle) * Constants.ROBOT_Y / 2000.0,
          0.0,
          pose.getRotation());
    }

    DriveSetpoints(Pose3d tag, boolean side) {
      double sideOffset = 0.32 / 2;

      Pose3d mappedPose = mapPose(tag);
      Rotation3d rotation = mappedPose.getRotation();
      double baseAngle = rotation.getAngle();
      double cos = Math.cos(baseAngle);
      double sin = Math.sin(baseAngle);
      double xOffset = sideOffset * sin;
      double yOffset = sideOffset * cos;
      if (side) {
        this.pose =
            new Pose2d(
                mappedPose.getX() + xOffset,
                mappedPose.getY() - yOffset,
                rotation.toRotation2d().plus(Rotation2d.kPi));
      } else {
        this.pose =
            new Pose2d(
                mappedPose.getX() - xOffset,
                mappedPose.getY() + yOffset,
                rotation.toRotation2d().plus(Rotation2d.kPi));
      }
    }

    DriveSetpoints(Pose2d pose) {
      this.pose = pose;
    }

    public static final Struct<DriveSetpoints> struct =
        StructGenerator.genEnum(DriveSetpoints.class);
  }
}
