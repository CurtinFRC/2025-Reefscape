package org.curtinfrc.frc2025.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.curtinfrc.frc2025.subsystems.leds.LEDsIO.LEDsColour;
import org.littletonrobotics.junction.Logger;

public class LEDs extends SubsystemBase {
  private final LEDsIO io;
  private final LEDsIOInputsAutoLogged inputs = new LEDsIOInputsAutoLogged();

  public LEDs(LEDsIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("LEDs", inputs);
  }

  public Command setPink() {
    return runOnce(() -> io.switchColor(LEDsColour.PINK));
  }

  public Command setGreen() {
    return runOnce(() -> io.switchColor(LEDsColour.GREEN));
  }

  public Command setBlue() {
    return runOnce(() -> io.switchColor(LEDsColour.BLUE));
  }
}
