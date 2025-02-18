package org.curtinfrc.frc2025.subsystems.ejector;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.curtinfrc.frc2025.Constants;
import org.curtinfrc.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Ejector extends SubsystemBase {
  private final EjectorIO io;
  private final EjectorIOInputsAutoLogged inputs = new EjectorIOInputsAutoLogged();
  private final PIDController pid = new PIDController(0, 0, 0);
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0);

  // Tunable Numbers
  private final LoggedTunableNumber kS = new LoggedTunableNumber("Ejector/kS");
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Ejector/kV");
  private final LoggedTunableNumber kP = new LoggedTunableNumber("Ejector/kP");
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Ejector/kD");

  public Ejector(EjectorIO io) {
    this.io = io;
    switch (Constants.robotType) {
      case COMPBOT:
        kS.initDefault(0.8);
        kV.initDefault(0.11);
        ff.setKs(kS.get());
        ff.setKv(kV.get());
        kP.initDefault(0);
        kD.initDefault(0);
        pid.setPID(kP.get(), 0, kD.get());
        break;
      case DEVBOT:
        kS.initDefault(0.8);
        kV.initDefault(0.11);
        ff.setKs(kS.get());
        ff.setKv(kV.get());
        kP.initDefault(0);
        kD.initDefault(0);
        pid.setPID(kP.get(), 0, kD.get());
        break;
      case SIMBOT:
        kS.initDefault(0.8);
        kV.initDefault(0.11);
        ff.setKs(kS.get());
        ff.setKv(kV.get());
        kP.initDefault(0);
        kD.initDefault(0);
        pid.setPID(kP.get(), 0, kD.get());
        break;
    }
  }

  public final Trigger sensor = new Trigger(() -> inputs.sensor);
  public final Trigger atSetpoint = new Trigger(pid::atSetpoint);

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Ejector", inputs);

    if (kS.hasChanged(hashCode())) {
      ff.setKs(kS.get());
    }

    if (kV.hasChanged(hashCode())) {
      ff.setKv(kV.get());
    }

    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      pid.setPID(kP.get(), 0, kD.get());
    }
  }

  public Command stop() {
    return runOnce(() -> io.setVoltage(0));
  }

  public Command eject(double rpm) {
    return run(
        () -> {
          Logger.recordOutput("Ejector/VelocitySetpoint", rpm);
          var pid_out = pid.calculate(inputs.angularVelocityRotationsPerMinute, rpm);
          Logger.recordOutput("Ejector/VelocityErrror", pid.getError());
          var ff_out = ff.calculate(rpm);
          io.setVoltage(pid_out + ff_out);
        });
  }
}
