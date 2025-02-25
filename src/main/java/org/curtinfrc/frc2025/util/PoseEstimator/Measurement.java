package org.curtinfrc.frc2025.util.PoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.Logger;

public class Measurement {
  protected Double time;

  protected Pose2dVector pose; // m
  protected Pose2dVector velocity; // m/s
  protected Pose2dVector acceleration; // m/s/s

  public Measurement() {
    pose = new Pose2dVector();
    velocity = new Pose2dVector();
    acceleration = new Pose2dVector();
    time = 0.;
  }

  public Measurement(Pose2d initialPose) {
    pose = new Pose2dVector(initialPose);
    velocity = new Pose2dVector();
    acceleration = new Pose2dVector();
    time = 0.;
  }

  public Measurement(
      Pose2dVector newPose,
      Pose2dVector newVelocity,
      Pose2dVector newAcceleration,
      Double newTime) {
    pose = newPose;
    velocity = newVelocity;
    acceleration = newAcceleration;
    time = newTime;
  }

  public void log(String key) {
    pose.log(key + "Pose/");
    velocity.log(key + "Velocity/");
    acceleration.log(key + "Acceleration/");
    Logger.recordOutput(key + "time", time);
  }

  double time() {
    return time;
  }

  public void physicsUpdate(Double currentTime) {
    Double dt = currentTime - time;

    velocity = velocity.plus(acceleration.times(dt));

    pose = pose.plus(velocity.times(dt));

    time = currentTime;
  }

  public Pose2d getPoseAsPose2d() {
    return pose.asPose2d();
  }

  public Measurement merge(Measurement with) {

    // should only merge if have the same time
    if (Math.abs(time - with.time) > 0.001) {
      System.out.println("Merge measurements with different times");
    }
    return new Measurement(
        pose.merge(with.pose),
        velocity.merge(with.velocity),
        acceleration.merge(with.acceleration),
        time);
  }
}
