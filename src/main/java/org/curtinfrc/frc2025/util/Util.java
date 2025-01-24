package org.curtinfrc.frc2025.util;

public final class Util {
  /**
   * Clamps a value to a specified inclusive range [min, max].
   *
   * @param min The minimum allowable value (inclusive).
   * @param max The maximum allowable value (inclusive).
   * @param value The value to clamp.
   * @return The clamped value.
   * @throws IllegalArgumentException if min > max.
   */
  public static double clamp(double min, double max, double value) {
    if (min > max) {
      throw new IllegalArgumentException(
          String.format("Invalid range: min (%.2f) cannot be greater than max (%.2f)", min, max));
    }
    return Math.min(max, Math.max(min, value));
  }
}
