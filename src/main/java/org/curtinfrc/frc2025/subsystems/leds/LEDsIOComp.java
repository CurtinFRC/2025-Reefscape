// pwm colours: https://1166281274-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2F-ME3KPEhFI6-MDoP9nZD%2Fuploads%2FMOYJvZmWgxCVKJhcV5fn%2FREV-11-1105-LED-Patterns.pdf?alt=media&token=e8227890-6dd3-498d-834a-752fa43413fe

package org.curtinfrc.frc2025.subsystems.leds;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LEDsIOComp implements LEDsIO{

    LEDsColour currentColour = LEDsColour.PINK;
    LEDsState currentState = LEDsState.STATIC;

    private final Spark LEDController = new Spark(18);

    @Override
    public void updateInputs(LEDsIOInputs inputs) {
        inputs.currentColour = currentColour;
        inputs.currentState = currentState;
        switch (currentColour) {
            case PINK:
                switch (currentState) {
                    case STATIC:
                        LEDController.set(0.57);
                        break;
        
                    case BLINK:
                        LEDController.set(0.15);
                        break;
                default:
                    break;
                }
                break;

            case GREEN:
                switch (currentState) {
                    case STATIC:
                        LEDController.set(0.77);
                        break;
        
                    case BLINK:
                        LEDController.set(0.35);
                        break;
                default:
                    break;
                }
                break;
        
            default:
                break;
        }
    }

    @Override
    public void switchColor(LEDsColour newColour) {
        currentColour = newColour;
    }

    @Override
    public void switchState(LEDsState newState) {
        currentState = newState;
    }

}
