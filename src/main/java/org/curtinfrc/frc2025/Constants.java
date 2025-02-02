// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.curtinfrc.frc2025;

import static org.curtinfrc.frc2025.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.List;
import org.curtinfrc.frc2025.util.PoseUtil;
import org.littletonrobotics.junction.Logger;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final RobotType robotType = RobotType.DEVBOT;
  public static final double ROBOT_X = 660; // mm
  public static final double ROBOT_Y = 680;

  public static final Mode getMode() {
    return switch (robotType) {
      case DEVBOT, COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  public static enum RobotType {
    /** Running in simulation */
    SIMBOT,

    /** Running competition robot. */
    COMPBOT,

    /** Running developer robot. */
    DEVBOT,
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static void main(String... args) {
    if (robotType == RobotType.SIMBOT) {
      System.out.println("Invalid robot type selected for deploy: SIMBOT.");
      System.exit(1);
    }
  }

  // TODO: MAKE SETPOINTS
  public enum Setpoints {
    /* in mm */
    COLLECT(0, List.of(13, 12), List.of(1, 2), false),
    // L1(460, List.of(17, 18, 19, 20, 21, 22), List.of(9, 8, 10, 8, 11, 6)), //TODO
    // TODO actually subtract
    L2(11, List.of(18 /*17, 18, 19, 20, 21, 22*/), List.of(9, 8, 10, 8, 11, 6), true),
    L3(32.8, List.of(18 /*17, 18, 19, 20, 21, 22*/), List.of(9, 8, 10, 8, 11, 6), true);

    private final double _elevatorSetpoint;
    private final List<Integer> _tagIdsBlue;
    private final List<Integer> _tagIdsRed;
    private final boolean flip;

    Setpoints(double elevator, List<Integer> tagIdsBlue, List<Integer> tagIdsRed, boolean flip) {
      this._elevatorSetpoint = elevator;
      this._tagIdsBlue = tagIdsBlue;
      this._tagIdsRed = tagIdsRed;
      this.flip = flip;
    }

    public double elevatorSetpoint() {
      return this._elevatorSetpoint;
    }

    public Pose3d toPose(Pose3d currentPose) {
      return resolvePose(
          DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
              ? _tagIdsBlue
              : _tagIdsRed,
          currentPose);
    }

    private Pose3d resolvePose(List<Integer> tagIds, Pose3d currentPose) {
      if (tagIds.isEmpty()) return new Pose3d();

      double sideOffset = 0.32 / 2;

      class ClosestPose {
        Pose3d pose = null;
        double distance = Double.MAX_VALUE;
      }

      ClosestPose closest = new ClosestPose();

      for (int tagId : tagIds) {
        aprilTagLayout
            .getTagPose(tagId)
            .ifPresent(
                tagPose -> {
                  Pose3d mappedPose = PoseUtil.mapPose(tagPose);
                  // mappedPose = mappedPose.rotateBy(new Rotation3d(0, 0, Math.PI));
                  Rotation3d rotation = mappedPose.getRotation();
                  double baseAngle = rotation.getAngle(),
                      cos = Math.cos(baseAngle),
                      sin = Math.sin(baseAngle);
                  double xOffset = sideOffset * sin, yOffset = sideOffset * cos;

                  for (Pose3d pose :
                      new Pose3d[] {
                        new Pose3d(
                            mappedPose.getX() + xOffset,
                            mappedPose.getY() - yOffset,
                            mappedPose.getZ(),
                            rotation.rotateBy(new Rotation3d(0, 0, this.flip ? Math.PI : 0))),
                        new Pose3d(
                            mappedPose.getX() - xOffset,
                            mappedPose.getY() + yOffset,
                            mappedPose.getZ(),
                            rotation.rotateBy(new Rotation3d(0, 0, this.flip ? Math.PI : 0)))
                      }) {
                    double distance = distanceBetween(pose, currentPose);
                    if (distance < closest.distance) {
                      closest.pose = pose;
                      closest.distance = distance;
                    }
                  }

                  String tagPrefix = String.format("resolvePose.tagId_%d", tagId);
                  Logger.recordOutput(tagPrefix + "/baseAngle", String.format("%.2f", baseAngle));
                  Logger.recordOutput(
                      tagPrefix + "/leftPose",
                      formatPose(
                          new Pose3d(
                              mappedPose.getX() + xOffset,
                              mappedPose.getY() - yOffset,
                              mappedPose.getZ(),
                              rotation)));
                  Logger.recordOutput(
                      tagPrefix + "/rightPose",
                      formatPose(
                          new Pose3d(
                              mappedPose.getX() - xOffset,
                              mappedPose.getY() + yOffset,
                              mappedPose.getZ(),
                              rotation)));
                  Logger.recordOutput(tagPrefix + "/currentPose", formatPose(currentPose));
                  Logger.recordOutput(tagPrefix + "/tagPose", formatPose(tagPose));
                });
      }
      return closest.pose != null ? closest.pose : new Pose3d();
    }

    private String formatPose(Pose3d pose) {
      return String.format("(%.2f, %.2f, %.2f)", pose.getX(), pose.getY(), pose.getZ());
    }

    private double distanceBetween(Pose3d pose1, Pose3d pose2) {
      var t1 = pose1.getTranslation();
      var t2 = pose2.getTranslation();
      return Math.sqrt(
          Math.pow(t1.getX() - t2.getX(), 2)
              + Math.pow(t1.getY() - t2.getY(), 2)
              + Math.pow(t1.getZ() - t2.getZ(), 2));
    }
  }
}
