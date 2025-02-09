package org.curtinfrc.frc2025.util;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;

import java.util.function.Supplier;

public class SparkUtil {
  /** Stores whether any error was has been detected by other utility methods. */
  public static boolean sparkStickyFault = false;

  public static void tryUntilOk(int maxAttempts, Supplier<REVLibError> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error == REVLibError.kOk) break;
    }
  }

  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error == REVLibError.kOk) {
        break;
      } else {
        sparkStickyFault = true;
      }
    }
  }
}
