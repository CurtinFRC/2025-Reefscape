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

<<<<<<< HEAD
=======
import static org.curtinfrc.frc2025.subsystems.vision.VisionConstants.aprilTagLayout;

import java.util.List;
import java.util.Optional;

import org.curtinfrc.frc2025.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
>>>>>>> 498261c (auto align)
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final RobotType robotType = RobotType.DEVBOT;

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
  // TODO: MAKE SETPOINTS  
  public enum Setpoints {
      /* in mm */
      NONE(-1, List.of(), List.of()),
      L1(460, List.of(17, 18, 19, 20, 21, 22), List.of(7, 8, 9, 10, 11, 12)),
      L2(810, List.of(17, 18, 19, 20, 21, 22), List.of(7, 8, 9, 10, 11, 12)),
      L3(1210, List.of(17, 18, 19, 20, 21, 22), List.of(7, 8, 9, 10, 11, 12)),
      COLLECT(950, List.of(13, 12), List.of(1, 2));
  
      private int elevator;
      private Pose3d pose;
  
      Setpoints(int elevator, List<Integer> tagIdsBlue, List<Integer> tagIdsRed) {
          this.elevator = elevator;
          this.pose = resolvePose(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? tagIdsBlue : tagIdsRed);
      }
  
      public int elevatorSetpoint() {
          return this.elevator;
      }
  
      public Pose3d toPose() {
          return this.pose;
      }
  
      private Pose3d resolvePose(List<Integer> tagIds) {
          if (tagIds.isEmpty()) {
              return null; // No valid pose
          }
  
          for (int tagId : tagIds) {
              Optional<Pose3d> pose = aprilTagLayout.getTagPose(tagId);
              if (pose.isPresent()) {
                  return pose.get(); // Return the first valid pose found
              }
          }
  
          return null; // No valid pose found
      }
  }
}
