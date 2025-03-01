package org.curtinfrc.frc2025.subsystems.led;

import org.curtinfrc.frc2025.subsystems.led.LEDIO.State;
import org.curtinfrc.frc2025.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class LED extends VirtualSubsystem {
  private final LEDIO io;
  private final LEDIOInputsAutoLogged inputs;

  public LED(LEDIO io) {
    this.io = io;
    this.inputs = new LEDIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("LED", inputs);
  }

  public void setState(State state) {
    io.setState(state);
  }
}
