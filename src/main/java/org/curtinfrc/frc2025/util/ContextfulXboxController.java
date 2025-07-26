package org.curtinfrc.frc2025.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class ContextfulXboxController {
  private final CommandXboxController xboxController;
  private final BooleanSupplier context;


  public ContextfulXboxController(int port, BooleanSupplier context) {
    xboxController = new CommandXboxController(port);
    this.context = context;
  }

  public Trigger a() {
    return xboxController.a().and(context);
  }

  public Trigger b() {
    return xboxController.b().and(context);
  }

  public Trigger x() {
    return xboxController.x().and(context);
  }

  public Trigger y() {
    return xboxController.y().and(context);
  }

  public Trigger leftBumper() {
    return xboxController.leftBumper().and(context);
  }

  public Trigger rightBumper() {
    return xboxController.rightBumper().and(context);
  }

  public Trigger back() {
    return xboxController.back().and(context);
  }

  public Trigger start() {
    return xboxController.start().and(context);
  }

  public Trigger leftStick() {
    return xboxController.leftStick().and(context);
  }

  public Trigger rightStick() {
    return xboxController.rightStick().and(context);
  }

  public Trigger leftTrigger() {
    return xboxController.leftTrigger().and(context);
  }

  public Trigger rightTrigger() {
    return xboxController.rightTrigger().and(context);
  }

  public double getLeftX() {
    return xboxController.getLeftX();
  }

  public double getRightX() {
    return xboxController.getRightX();
  }

  public double getLeftY() {
    return xboxController.getLeftY();
  }

  public double getRightY() {
    return xboxController.getRightY();
  }

  public double getLeftTriggerAxis() {
    return xboxController.getLeftTriggerAxis();
  }

  public double getRightTriggerAxis() {
    return xboxController.getRightTriggerAxis();
  }

  public void setRumble(RumbleType type, double value) {
    xboxController.setRumble(type, value);
  }

  public Trigger povUp() {
    return xboxController.povUp();
  }

  public Trigger povUpRight() {
    return xboxController.povUpRight();
  }

  public Trigger povRight() {
    return xboxController.povRight();
  }

  public Trigger povDownRight() {
    return xboxController.povDownRight();
  } 

  public Trigger povDown() {
    return xboxController.povDown();
  }

  public Trigger povDownLeft() {
    return xboxController.povDownLeft();
  }

  public Trigger povLeft() {
    return xboxController.povLeft();
  }

  public Trigger povUpLeft() {
    return xboxController.povUpLeft();
  }

  public boolean isConnected() {
    return xboxController.isConnected();
  }
}