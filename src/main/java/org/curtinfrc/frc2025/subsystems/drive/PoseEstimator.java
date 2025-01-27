package org.curtinfrc.frc2025.subsystems.drive;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotController;
import java.util.LinkedList;
import java.util.OptionalInt;

public class PoseEstimator {
  private static final double kMaxSampleAge = 0.3;

  private final class TimestampedTwist2d extends Twist2d {
    public final double timestamp;

    public TimestampedTwist2d(double dx, double dy, double dtheta, double timestamp) {
      super(dx, dy, dtheta);
      this.timestamp = timestamp;
    }
  }

  private final LinkedList<TimestampedTwist2d> samples = new LinkedList<>();
  private Pose2d rootPose = new Pose2d();
  private final SwerveModulePosition[] prevWheelPositions;
  private final SwerveDriveKinematics kinematics;
  private Rotation2d gyroOffset;
  private Rotation2d prevAngle;

  public PoseEstimator(
      SwerveDriveKinematics kinematics,
      SwerveModulePosition[] wheelPositions,
      Rotation2d gyroAngle) {
    this.kinematics = kinematics;
    prevWheelPositions = kinematics.copy(wheelPositions);
    gyroOffset = rootPose.getRotation().minus(gyroAngle);
    prevAngle = rootPose.getRotation();
  }

  public void resetPosition(
      Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions, Pose2d pose) {
    rootPose = pose;
    prevAngle = pose.getRotation();
    gyroOffset = pose.getRotation().minus(gyroAngle);
    kinematics.copyInto(wheelPositions, prevWheelPositions);
    samples.clear();
  }

  private OptionalInt indexForTimestamp(double timestamp) {
    if (samples.isEmpty()) {
      return OptionalInt.empty();
    }

    if (samples.getFirst().timestamp > timestamp) {
      return OptionalInt.empty();
    }
    if (samples.getLast().timestamp < timestamp) {
      return OptionalInt.of(samples.size() - 1);
    }
    int low = 0;
    int high = samples.size() - 1;
    while (low <= high) {
      int mid = (low + high) / 2;
      double midTime = samples.get(mid).timestamp;
      if (midTime < timestamp) {
        low = mid + 1;
      } else if (midTime > timestamp) {
        high = mid - 1;
      } else {
        return OptionalInt.of(mid);
      }
    }
    return OptionalInt.of(low - 1);
  }

  private Pose2d poseAtIndex(int index) {
    // switching this over to primitive math would be a good idea
    Pose2d pose = rootPose;
    for (int i = 0; i < index; i++) {
      pose = pose.exp(samples.get(i));
    }
    return pose;
  }

  private void pruneToRoot() {
    while (!samples.isEmpty()
        && samples.getFirst().timestamp < RobotController.getTime() - kMaxSampleAge) {
      rootPose = rootPose.exp(samples.removeFirst());
    }
  }

  /**
   * Adds a sample to the estimator
   *
   * @param pose the pose of the robot at the time of the sample
   * @param timestamp the timestamp of the sample
   * @param weight the weight of the sample (0.0 to 1.0)
   */
  public void addVisionMeasurement(Pose2d pose, double timestamp, Vector<N3> weight) {
    var opt = indexForTimestamp(timestamp);
    if (opt.isEmpty()) {
      // timestamp is before the first sample
      return;
    }
    int index = opt.getAsInt();

    Pose2d lastPose = poseAtIndex(index);
    Twist2d twist = lastPose.log(pose);
    samples.add(
        index,
        new TimestampedTwist2d(
            twist.dx * weight.get(0),
            twist.dy * weight.get(1),
            twist.dtheta * weight.get(2),
            timestamp));
    pruneToRoot();
  }

  public void updateWithTime(
      SwerveModulePosition[] wheelPositions,
      Rotation2d gyroAngle,
      double timestamp,
      Vector<N3> weight) {
    var angle = gyroAngle.plus(gyroOffset);
    var twist = kinematics.toTwist2d(prevWheelPositions, wheelPositions);
    twist.dtheta = angle.minus(prevAngle).getRadians();

    samples.add(
        new TimestampedTwist2d(
            twist.dx * weight.get(0),
            twist.dy * weight.get(1),
            twist.dtheta * weight.get(2),
            timestamp));

    kinematics.copyInto(wheelPositions, prevWheelPositions);
    prevAngle = angle;
    pruneToRoot();
  }

  public Pose2d getEstimatedPose() {
    return poseAtIndex(samples.size());
  }
}
