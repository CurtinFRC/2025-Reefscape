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

package org.curtinfrc.frc2025.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.RobotController;
import java.util.function.Supplier;

public class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  /**
   * Converts a time from Phoenix6's timebase to FPGA time. Should only be used inside of an IO
   * layer as it relies on raw FPGA time.
   */
  public static double phoenixToFPGATime(double timeSeconds) {
    return (RobotController.getFPGATime() - Utils.getCurrentTimeSeconds()) + timeSeconds;
  }

  /**
   * Converts a time from Phoenix6's timebase to the robots timebase. Can be used outside of IO
   * layers.
   */
  public static double phoenixToRobotTime(double timeSeconds) {
    return (RobotController.getTime() - Utils.getCurrentTimeSeconds()) + timeSeconds;
  }
}
