// pwm colours:
// https://1166281274-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2F-ME3KPEhFI6-MDoP9nZD%2Fuploads%2FMOYJvZmWgxCVKJhcV5fn%2FREV-11-1105-LED-Patterns.pdf?alt=media&token=e8227890-6dd3-498d-834a-752fa43413fe

package org.curtinfrc.frc2025.subsystems.leds;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LEDsIOComp implements LEDsIO {

  LEDsColour currentColour = LEDsColour.PINK;

  private final Spark LEDController = new Spark(1);

  @Override
  public void updateInputs(LEDsIOInputs inputs) {
    inputs.currentColour = currentColour;
    switch (currentColour) {
      case PINK:
        LEDController.set(0.57);
        break;

      case GREEN:
        LEDController.set(0.77);
        break;

      case BLUE:
        LEDController.set(0.83);
        break;

      default:
        break;
    }
  }

  @Override
  public void switchColor(LEDsColour newColour) {
    currentColour = newColour;
  }
}
