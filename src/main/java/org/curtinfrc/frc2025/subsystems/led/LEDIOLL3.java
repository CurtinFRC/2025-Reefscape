package org.curtinfrc.frc2025.subsystems.led;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LEDIOLL3 implements LEDIO {
  private static final String LL_NAME = "limelight-3";
  private final NetworkTableEntry mode;
  private State state = State.Inactive;

  public LEDIOLL3() {
    mode = NetworkTableInstance.getDefault().getEntry(LL_NAME);
  }

  @Override
  public void updateInputs(LEDIOInputs inputs) {
    inputs.state = state;
    switch (inputs.state) {
      case Active:
        mode.setNumber(3);
        break;
      case Blink:
        mode.setNumber(2);
        break;
      case Inactive:
        mode.setNumber(1);
    }
  }
}
