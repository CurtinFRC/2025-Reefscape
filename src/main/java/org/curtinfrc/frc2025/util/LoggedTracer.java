package org.curtinfrc.frc2025.util;

import edu.wpi.first.wpilibj.Timer;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/** Utility class for logging code execution times. */
public class LoggedTracer {
  private LoggedTracer() {}

  private static Map<String, Double> epochs = new HashMap<>();

  /** Reset the clock. */
  public static void reset(String epochName) {
    if (epochs.containsKey(epochName)) {
      epochs.replace(epochName, Timer.getFPGATimestamp());
    } else {
      epochs.put(epochName, Timer.getFPGATimestamp());
    }
  }

  /** Save the time elapsed since the last reset or record. */
  public static void record(String epochName) {
    double now = Timer.getFPGATimestamp();
    Logger.recordOutput("LoggedTracer/" + epochName + "MS", (now - epochs.get(epochName)) * 1000.0);
    epochs.replace(epochName, now);
  }
}
