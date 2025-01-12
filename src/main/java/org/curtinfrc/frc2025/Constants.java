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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.List;
import java.util.Optional;

import org.curtinfrc.frc2025.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
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

  @SuppressWarnings("resource")
  public static RobotType getRobot() {
    if (RobotBase.isReal() && RobotType.SIMBOT == robotType) {
      new Alert("Robot type is invalid, defaulting to competition robot.", AlertType.kWarning);
      return RobotType.COMPBOT;
    }
    return robotType;
  }

  // public static Alliance getAlliance() {
  //   return getMode() == Mode.SIM ? (List.of(AllianceStationID.Blue1, AllianceStationID.Blue2,
  // AllianceStationID.Blue3).contains(DriverStationSim.getAllianceStationId()) ? Alliance.Blue :
  // Alliance.Red) : DriverStation.getAlliance().get();
  // }

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
    NONE(-1, List.of(), List.of()),
    COLLECT(950, List.of(13, 12), List.of(1, 2)),
    L1(460, List.of(17, 18, 19, 20, 21, 22), List.of(9, 8, 10, 8, 11, 6)),
    L2(810, List.of(17, 18, 19, 20, 21, 22), List.of(9, 8, 10, 8, 11, 6)),
    L3(1210, List.of(17, 18, 19, 20, 21, 22), List.of(9, 8, 10, 8, 11, 6));

    private final int _elevator;
    private final List<Integer> _tagIdsBlue;
    private final List<Integer> _tagIdsRed;
    private Pose3d _pose = null;

    Setpoints(int elevator, List<Integer> tagIdsBlue, List<Integer> tagIdsRed) {
      this._elevator = elevator;
      this._tagIdsBlue = tagIdsBlue;
      this._tagIdsRed = tagIdsRed;
    }

    public int elevatorSetpoint() {
      return this._elevator;
    }

    public Pose3d toPose() {
      return resolvePose(
          DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
              ? _tagIdsRed
              : _tagIdsBlue);
    }

    private Pose3d resolvePose(List<Integer> tagIds) {
      if (tagIds.isEmpty()) {
        return new Pose3d(); // No valid pose
      }

      for (int tagId : tagIds) {
        Optional<Pose3d> pose = aprilTagLayout.getTagPose(tagId);
        if (pose.isPresent()) {
          return pose.get(); // Return the first valid pose found
        }
      }

      return new Pose3d(-1, -1, 1, new Rotation3d()); // No valid pose found
    }
  }
}
