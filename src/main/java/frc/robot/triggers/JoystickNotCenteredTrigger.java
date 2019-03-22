/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.triggers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.buttons.Trigger;

/**
 * Returns whether a joystick is within a deadband of 0
 */
public class JoystickNotCenteredTrigger extends Trigger {

  private Joystick stick;
  private AxisType axis;
  private double deadband;

  public JoystickNotCenteredTrigger(Joystick stick, AxisType axis, double deadband) {
    this.stick = stick;
    this.axis = axis;
    this.deadband = deadband;
  }

  @Override
  public boolean get() {
    return Math.abs(stick.getAxis(axis)) <= deadband;
  }
}
