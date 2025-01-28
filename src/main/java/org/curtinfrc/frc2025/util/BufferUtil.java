package org.curtinfrc.frc2025.util;

import edu.wpi.first.util.CircularBuffer;

public class BufferUtil {
  private BufferUtil() {}

  public static Double average(CircularBuffer<Double> buffer) {
    Double total = 0.0;
    for (var i = 0; i < buffer.size(); i++) {
      total += buffer.get(i);
    }
    return total;
  }
}
