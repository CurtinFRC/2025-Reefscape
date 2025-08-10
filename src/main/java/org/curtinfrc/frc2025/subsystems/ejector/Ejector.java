package org.curtinfrc.frc2025.subsystems.ejector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Ejector extends SubsystemBase {
  private final EjectorIO io;
  private final EjectorIOInputsAutoLogged inputs = new EjectorIOInputsAutoLogged();

  public Ejector(EjectorIO io) {
    this.io = io;
  }

  @AutoLogOutput(key = "Ejector/HasTried")
  private boolean hasTried = false;

  @AutoLogOutput(key = "Ejector/HasContacted")
  private boolean hasContacted = false;

  @AutoLogOutput(key = "Ejector/HasPopped")
  public final Trigger hasPopped =
      new Trigger(
          () -> {
            if (hasContacted) {
              hasTried = false;
              var ret = inputs.angularVelocityRotationsPerSecond > 50 && inputs.currentAmps < 45;
              if (ret) {
                hasContacted = false;
              }
              return ret;
            }

            if (hasTried) {
              hasContacted =
                  inputs.angularVelocityRotationsPerSecond < 45 && inputs.currentAmps > 45;
            }

            if (!hasTried && !hasContacted) {
              hasTried = inputs.angularVelocityRotationsPerSecond > 50 && inputs.currentAmps > 2;
            }

            return false;
          });

  public final Trigger backSensor = new Trigger(() -> inputs.backSensor);
  public final Trigger frontSensor = new Trigger(() -> inputs.frontSensor);

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Ejector", inputs);
  }

  public Command stop() {
    return runOnce(() -> io.setVoltage(0));
  }

  public Command eject(double rotationsPerSecond) {
    return run(() -> {
          io.setVelocity(rotationsPerSecond);
        })
        .withName("Eject");
  }

  public Command setVoltage(double volts) {
    return run(() -> io.setVoltage(volts));
  }
}
