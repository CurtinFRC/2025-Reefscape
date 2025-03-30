package org.curtinfrc.frc2025.subsystems.drive;

import static org.curtinfrc.frc2025.subsystems.vision.VisionConstants.aprilTagLayout;

import choreo.util.ChoreoAllianceFlipUtil;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;
import java.util.function.Supplier;
import org.curtinfrc.frc2025.Constants;
import org.curtinfrc.frc2025.generated.CompTunerConstants;

public final class DriveConstants {
  public static final double coralOffset = 0.32 / 2;
  public static final double algaeOffset = 0.155;
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

  public static enum DriveSetpoints implements StructSerializable {
    A(aprilTagLayout.getTagPose(18).get(), true, coralOffset),
    B(aprilTagLayout.getTagPose(18).get(), false, coralOffset),
    C(aprilTagLayout.getTagPose(17).get(), true, coralOffset),
    D(aprilTagLayout.getTagPose(17).get(), false, coralOffset),
    E(aprilTagLayout.getTagPose(22).get(), true, coralOffset),
    F(aprilTagLayout.getTagPose(22).get(), false, coralOffset),
    G(aprilTagLayout.getTagPose(21).get(), true, coralOffset),
    H(aprilTagLayout.getTagPose(21).get(), false, coralOffset),
    I(aprilTagLayout.getTagPose(20).get(), true, coralOffset),
    J(aprilTagLayout.getTagPose(20).get(), false, coralOffset),
    K(aprilTagLayout.getTagPose(19).get(), true, coralOffset),
    L(aprilTagLayout.getTagPose(19).get(), false, coralOffset),
    CLOSE(aprilTagLayout.getTagPose(18).get(), true, algaeOffset),
    CLOSE_LEFT(aprilTagLayout.getTagPose(19).get(), true, algaeOffset),
    CLOSE_RIGHT(aprilTagLayout.getTagPose(17).get(), true, algaeOffset),
    FAR_RIGHT(aprilTagLayout.getTagPose(22).get(), true, algaeOffset),
    FAR_LEFT(aprilTagLayout.getTagPose(20).get(), true, algaeOffset),
    FAR(aprilTagLayout.getTagPose(21).get(), true, algaeOffset),
    LEFT_HP(
        new Pose2d(1.148711085319519, 7.199769020080566, Rotation2d.fromDegrees(125.989 + 180))),
    RIGHT_HP(
        new Pose2d(
            0.9220133423805237,
            0.9964936375617981,
            Rotation2d.fromDegrees(125.989 + 180).unaryMinus()));

    private final Pose2d pose;

    public Pose2d getPose() {
      boolean isFlipped =
          DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
      if (isFlipped) {
        return ChoreoAllianceFlipUtil.flip(pose);
      }
      // if (isFlipped) {
      //   var partialFlipped = ChoreoAllianceFlipUtil.flip(pose);
      //   return new Pose2d(partialFlipped.getX(), Flipper.MIRRORED.flipY(partialFlipped.getY()),
      // partialFlipped.getRotation().unaryMinus());
      // }
      return pose;
    }

    static Pose3d mapPose(Pose3d pose) {
      double angle = pose.getRotation().getAngle();
      return new Pose3d(
          pose.getX() + Math.cos(angle) * Constants.ROBOT_X / 2,
          pose.getY() + Math.sin(angle) * Constants.ROBOT_Y / 2,
          0.0,
          pose.getRotation());
    }

    DriveSetpoints(Pose3d tag, boolean side, double sideOffset) {
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

    public static DriveSetpoints closest(Supplier<Pose2d> currentPose, List<Pose2d> possible) {
      var desiredPose = currentPose.get().nearest(possible);
      if (desiredPose.equals(A.getPose())) {
        return A;
      }
      if (desiredPose.equals(B.getPose())) {
        return B;
      }
      if (desiredPose.equals(C.getPose())) {
        return C;
      }
      if (desiredPose.equals(D.getPose())) {
        return D;
      }
      if (desiredPose.equals(E.getPose())) {
        return E;
      }
      if (desiredPose.equals(F.getPose())) {
        return F;
      }
      if (desiredPose.equals(G.getPose())) {
        return G;
      }
      if (desiredPose.equals(H.getPose())) {
        return H;
      }
      if (desiredPose.equals(I.getPose())) {
        return I;
      }
      if (desiredPose.equals(J.getPose())) {
        return J;
      }
      if (desiredPose.equals(K.getPose())) {
        return K;
      }
      if (desiredPose.equals(L.getPose())) {
        return L;
      }
      if (desiredPose.equals(FAR.getPose())) {
        return FAR;
      }
      if (desiredPose.equals(CLOSE.getPose())) {
        return CLOSE;
      }
      if (desiredPose.equals(FAR_RIGHT.getPose())) {
        return FAR_RIGHT;
      }
      if (desiredPose.equals(CLOSE_RIGHT.getPose())) {
        return CLOSE_RIGHT;
      }
      if (desiredPose.equals(FAR_LEFT.getPose())) {
        return FAR_LEFT;
      }
      if (desiredPose.equals(CLOSE_LEFT.getPose())) {
        return CLOSE_LEFT;
      }
      throw new IllegalStateException("No matching setpoint found!");
    }
  }
}
