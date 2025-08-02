package org.curtinfrc.frc2025.subsystems.popper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Popper extends SubsystemBase {
  private final PopperIO io;
  private final PopperIOInputsAutoLogged inputs = new PopperIOInputsAutoLogged();

  public Popper(PopperIO io) {
    this.io = io;
  }

  @AutoLogOutput(key = "Popper/HasTried")
  private boolean hasTried = false;

  @AutoLogOutput(key = "Popper/HasContacted")
  private boolean hasContacted = false;

  @AutoLogOutput(key = "Popper/HasPopped")
  public final Trigger hasPopped =
      new Trigger(
          () -> {
            if (hasContacted) {
              hasTried = false;
              var ret = inputs.angularVelocityRotationsPerSecond > 20 && inputs.currentAmps < 45;
              if (ret) {
                hasContacted = false;
              }
              return ret;
            }

            if (hasTried) {
              hasContacted =
                  inputs.angularVelocityRotationsPerSecond < 20 && inputs.currentAmps > 45;
            }

            return false;
          });



  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Popper", inputs);
  }

  public Command stop() {
    return runOnce(() -> io.setVoltage(0));
  }

  public Command pop(double rotationsPerSecond) {
    return runOnce(() -> hasTried = true).AndThen(run(() -> {
          io.setVelocity(rotationsPerSecond);
        })).finallyDo(() -> hasTried = false)
        .withName("Pop");
  }

  public Command setVoltage(double volts) {
    return run(() -> io.setVoltage(volts));
  }
}
