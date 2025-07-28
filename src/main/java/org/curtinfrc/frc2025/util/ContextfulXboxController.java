package org.curtinfrc.frc2025.util;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

public class ContextfulXboxController {
  private final CommandXboxController xboxController;
  private final BooleanSupplier context;

  public ContextfulXboxController(int port, BooleanSupplier context) {
    xboxController = new CommandXboxController(port);
    this.context = context;
  }

  /* All the Triggers as well as setRumble check for the context as they should
   * activate based on the controller state. None of the methods that return a
   * (non-trigger) value don't check the context as they return values from the
   * physical controller which isn't dependent on the context.
   */

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

  public Trigger povUp() {
    return xboxController.povUp().and(context);
  }

  public Trigger povUpRight() {
    return xboxController.povUpRight().and(context);
  }

  public Trigger povRight() {
    return xboxController.povRight().and(context);
  }

  public Trigger povDownRight() {
    return xboxController.povDownRight().and(context);
  }

  public Trigger povDown() {
    return xboxController.povDown().and(context);
  }

  public Trigger povDownLeft() {
    return xboxController.povDownLeft().and(context);
  }

  public Trigger povLeft() {
    return xboxController.povLeft().and(context);
  }

  public Trigger povUpLeft() {
    return xboxController.povUpLeft().and(context);
  }

  public Trigger pov(int angle) {
    return xboxController.pov(angle).and(context);
  }

  public Trigger pov(int pov, int angle, EventLoop loop) {
    return xboxController.pov(pov, angle, loop).and(context);
  }

  public Trigger povCentre() {
    return xboxController.povCenter().and(context);
  }

  public Trigger povCenter() {
    return xboxController.povCenter().and(context);
  }

  public Trigger axisLessThan(int axis, double threshold) {
    return xboxController.axisLessThan(axis, threshold).and(context);
  }

  public Trigger axisLessThan(int axis, double threshold, EventLoop loop) {
    return xboxController.axisLessThan(axis, threshold, loop).and(context);
  }

  public Trigger axisGreaterThan(int axis, double threshold) {
    return xboxController.axisGreaterThan(axis, threshold).and(context);
  }

  public Trigger axisGreaterThan(int axis, double threshold, EventLoop loop) {
    return xboxController.axisGreaterThan(axis, threshold, loop).and(context);
  }

  public Trigger axisMagnitudeGreaterThan(int axis, double threshold) {
    return xboxController.axisMagnitudeGreaterThan(axis, threshold).and(context);
  }

  public Trigger axisMagnitudeGreaterThan(int axis, double threshold, EventLoop loop) {
    return xboxController.axisMagnitudeGreaterThan(axis, threshold, loop).and(context);
  }

  public void setRumble(RumbleType type, double value) {
    xboxController.setRumble(type, value);
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

  public boolean isConnected() {
    return xboxController.isConnected();
  }

  public double getRawAxis(int axis) {
    return xboxController.getRawAxis(axis);
  }
}
