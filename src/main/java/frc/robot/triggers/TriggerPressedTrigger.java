/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.triggers;

import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * Returns whether a controller trigger is fully pressed
 */
public class TriggerPressedTrigger extends Trigger {

  private XboxController controller;
  private Hand hand;

  public TriggerPressedTrigger(XboxController controller, Hand hand) {
    this.controller = controller;
    this.hand = hand;
  }

  @Override
  public boolean get() {
    return controller.getTriggerAxis(hand) == 1;
  }
}
