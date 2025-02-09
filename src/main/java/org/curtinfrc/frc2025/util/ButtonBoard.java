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

  public Trigger CoralAB() {
    return button(Button.kAB.value);
  }

  public Trigger CoralCD() {
    return button(Button.kCD.value);
  }

  public Trigger CoralEF() {
    return button(Button.kEF.value);
  }

  public Trigger CoralGH() {
    return button(Button.kGH.value);
  }

  public Trigger CoralIJ() {
    return button(Button.kIJ.value);
  }

  public Trigger CoralKL() {
    return button(Button.kKL.value);
  }

  public Trigger Proccessor() {
    return button(Button.kProcessor.value);
  }

  public Trigger Left() {
    return button(Button.kPlayerLeft.value);
  }

  public Trigger Right() {
    return button(Button.kPlayerRight.value);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("HID");
    builder.publishConstString("ControllerType", "ButtonBoard");
  }
}

//   /**
//    * Constructs a Trigger instance around the left stick button's digital signal.
//    *
//    * @return a Trigger instance representing the left stick button's digital signal attached
//    *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
//    * @see #leftStick(EventLoop)
//    */
//   public Trigger leftStick() {
//     return leftStick(CommandScheduler.getInstance().getDefaultButtonLoop());
//   }

//   /**
//    * Constructs a Trigger instance around the left stick button's digital signal.
//    *
//    * @param loop the event loop instance to attach the event to.
//    * @return a Trigger instance representing the left stick button's digital signal attached
//    *     to the given loop.
//    */
//   public Trigger leftStick(EventLoop loop) {
//     return button(XboxController.Button.kLeftStick.value, loop);
//   }

//   /**
//    * Constructs a Trigger instance around the right stick button's digital signal.
//    *
//    * @return a Trigger instance representing the right stick button's digital signal attached
//    *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
//    * @see #rightStick(EventLoop)
//    */
//   public Trigger rightStick() {
//     return rightStick(CommandScheduler.getInstance().getDefaultButtonLoop());
//   }

//   /**
//    * Constructs a Trigger instance around the right stick button's digital signal.
//    *
//    * @param loop the event loop instance to attach the event to.
//    * @return a Trigger instance representing the right stick button's digital signal attached
//    *     to the given loop.
//    */
//   public Trigger rightStick(EventLoop loop) {
//     return button(XboxController.Button.kRightStick.value, loop);
//   }

//   /**
//    * Constructs a Trigger instance around the axis value of the left trigger. The returned
//    * trigger will be true when the axis value is greater than {@code threshold}.
//    *
//    * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This
// value
//    *     should be in the range [0, 1] where 0 is the unpressed state of the axis.
//    * @param loop the event loop instance to attach the Trigger to.
//    * @return a Trigger instance that is true when the left trigger's axis exceeds the provided
//    *     threshold, attached to the given event loop
//    */
//   public Trigger leftTrigger(double threshold, EventLoop loop) {
//     return axisGreaterThan(XboxController.Axis.kLeftTrigger.value, threshold, loop);
//   }

//   /**
//    * Constructs a Trigger instance around the axis value of the left trigger. The returned
//    * trigger will be true when the axis value is greater than {@code threshold}.
//    *
//    * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This
// value
//    *     should be in the range [0, 1] where 0 is the unpressed state of the axis.
//    * @return a Trigger instance that is true when the left trigger's axis exceeds the provided
//    *     threshold, attached to the {@link CommandScheduler#getDefaultButtonLoop() default
// scheduler
//    *     button loop}.
//    */
//   public Trigger leftTrigger(double threshold) {
//     return leftTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
//   }

//   /**
//    * Constructs a Trigger instance around the axis value of the left trigger. The returned
// trigger
//    * will be true when the axis value is greater than 0.5.
//    *
//    * @return a Trigger instance that is true when the left trigger's axis exceeds 0.5, attached
// to
//    *     the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
//    */
//   public Trigger leftTrigger() {
//     return leftTrigger(0.5);
//   }

//   /**
//    * Constructs a Trigger instance around the axis value of the right trigger. The returned
//    * trigger will be true when the axis value is greater than {@code threshold}.
//    *
//    * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This
// value
//    *     should be in the range [0, 1] where 0 is the unpressed state of the axis.
//    * @param loop the event loop instance to attach the Trigger to.
//    * @return a Trigger instance that is true when the right trigger's axis exceeds the provided
//    *     threshold, attached to the given event loop
//    */
//   public Trigger rightTrigger(double threshold, EventLoop loop) {
//     return axisGreaterThan(XboxController.Axis.kRightTrigger.value, threshold, loop);
//   }

//   /**
//    * Constructs a Trigger instance around the axis value of the right trigger. The returned
//    * trigger will be true when the axis value is greater than {@code threshold}.
//    *
//    * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This
// value
//    *     should be in the range [0, 1] where 0 is the unpressed state of the axis.
//    * @return a Trigger instance that is true when the right trigger's axis exceeds the provided
//    *     threshold, attached to the {@link CommandScheduler#getDefaultButtonLoop() default
// scheduler
//    *     button loop}.
//    */
//   public Trigger rightTrigger(double threshold) {
//     return rightTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
//   }

//   /**
//    * Constructs a Trigger instance around the axis value of the right trigger. The returned
// trigger
//    * will be true when the axis value is greater than 0.5.
//    *
//    * @return a Trigger instance that is true when the right trigger's axis exceeds 0.5, attached
// to
//    *     the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
//    */
//   public Trigger rightTrigger() {
//     return rightTrigger(0.5);
//   }

//   /**
//    * Get the X axis value of left side of the controller. Right is positive.
//    *
//    * @return The axis value.
//    */
//   public double getLeftX() {
//     return m_hid.getLeftX();
//   }

//   /**
//    * Get the X axis value of right side of the controller. Right is positive.
//    *
//    * @return The axis value.
//    */
//   public double getRightX() {
//     return m_hid.getRightX();
//   }

//   /**
//    * Get the Y axis value of left side of the controller. Back is positive.
//    *
//    * @return The axis value.
//    */
//   public double getLeftY() {
//     return m_hid.getLeftY();
//   }

//   /**
//    * Get the Y axis value of right side of the controller. Back is positive.
//    *
//    * @return The axis value.
//    */
//   public double getRightY() {
//     return m_hid.getRightY();
//   }

//   /**
//    * Get the left trigger axis value of the controller. Note that this axis is bound to the
//    * range of [0, 1] as opposed to the usual [-1, 1].
//    *
//    * @return The axis value.
//    */
//   public double getLeftTriggerAxis() {
//     return m_hid.getLeftTriggerAxis();
//   }

//   /**
//    * Get the right trigger axis value of the controller. Note that this axis is bound to the
//    * range of [0, 1] as opposed to the usual [-1, 1].
//    *
//    * @return The axis value.
//    */
//   public double getRightTriggerAxis() {
//     return m_hid.getRightTriggerAxis();
//   }
// }
