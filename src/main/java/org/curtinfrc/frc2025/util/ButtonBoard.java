package org.curtinfrc.frc2025.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonBoard extends CommandGenericHID implements Sendable {
  public enum Button {
    kAB(5),
    kCD(8),
    kEF(9),
    kGH(6),
    kIJ(10),
    kKL(12),
    kProcessor(7),
    kPlayerLeft(11),
    kPlayerRight(13);

    public final int value;

    Button(int value) {
      this.value = value;
    }
  }

  public ButtonBoard(final int port) {
    super(port);
  }

  public Trigger coralAB() {
    return button(Button.kAB.value);
  }

  public Trigger coralCD() {
    return button(Button.kCD.value);
  }

  public Trigger coralEF() {
    return button(Button.kEF.value);
  }

  public Trigger coralGH() {
    return button(Button.kGH.value);
  }

  public Trigger coralIJ() {
    return button(Button.kIJ.value);
  }

  public Trigger coralKL() {
    return button(Button.kKL.value);
  }

  public Trigger proccessor() {
    return button(Button.kProcessor.value);
  }

  public Trigger left() {
    return button(Button.kPlayerLeft.value);
  }

  public Trigger right() {
    return button(Button.kPlayerRight.value);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("HID");
    builder.publishConstString("ControllerType", "ButtonBoard");
  }
}
