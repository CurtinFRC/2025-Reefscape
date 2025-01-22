package org.curtinfrc.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import java.util.function.BooleanSupplier;

public class IntakeIOBeamBreak implements IntakeIO {
  private final DigitalInput intakeBeamBreak = new DigitalInput(99);
  public BooleanSupplier beamBroken = () -> !intakeBeamBreak.get();
  public BooleanSupplier beamConnected = () -> intakeBeamBreak.get();

  public IntakeIOBeamBreak() {}

  @Override
  public void updateInputs(IntakeIOInputs inputs) {}
}
