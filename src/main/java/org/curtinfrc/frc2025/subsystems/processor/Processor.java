package org.curtinfrc.frc2025.subsystems.processor;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Processor extends SubsystemBase {
    private final ProcessorIO io;
    private final ProcessorIOInputsAutoLogged inputs = new ProcessorIOInputsAutoLogged();
    
    public Processor(ProcessorIO io) {
        this.io = io;
    }

    // public final Trigger processorSensor = new Trigger(() -> inputs.processorSensor);

    @Override
    public void periodic() {}

      public Command stop() {
        return run(() -> io.setVoltage(0));
      }

      public Command intake(double volts) {
        return run(() -> io.setVoltage(volts));
      }

      public Command intake() {
        return run(() -> io.setVoltage(3));
      }
    }