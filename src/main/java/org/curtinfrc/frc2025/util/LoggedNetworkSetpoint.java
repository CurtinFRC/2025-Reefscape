package org.curtinfrc.frc2025.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructEntry;
import org.curtinfrc.frc2025.Constants.Setpoint;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;

// TODO sub choosers
public class LoggedNetworkSetpoint extends LoggedNetworkInput {
  private final String key;
  private final Setpoint defaultValue;
  private Setpoint value;
  private final StructEntry<Setpoint> entry;

  public LoggedNetworkSetpoint(String key, Setpoint defaultValue) {
    this.key = key;
    this.defaultValue = defaultValue;
    this.entry =
        NetworkTableInstance.getDefault()
            .getTable(key)
            .getStructTopic("Drive", Setpoint.struct)
            .getEntry(defaultValue);
    this.value = defaultValue;
    Logger.registerDashboardInput(this);
    this.entry.set(defaultValue);
    periodic();
  }

  public void set(Setpoint value) {
    this.value = value;
  }

  public Setpoint get() {
    return value;
  }

  private final LoggableInputs inputs =
      new LoggableInputs() {
        public void toLog(org.littletonrobotics.junction.LogTable table) {
          table.put(removeSlash(key), Setpoint.struct, value);
        }

        public void fromLog(org.littletonrobotics.junction.LogTable table) {
          value = table.get(removeSlash(key), Setpoint.struct, defaultValue);
        }
      };

  @Override
  public void periodic() {
    if (!Logger.hasReplaySource()) {
      value = entry.get(defaultValue);
    }
    Logger.processInputs(prefix, inputs);
  }
}
