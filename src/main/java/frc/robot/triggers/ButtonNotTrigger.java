/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.triggers;

import edu.wpi.first.wpilibj.buttons.Trigger;

/**
 * Matches the state of one trigger only if another trigger is not active
 */
public class ButtonNotTrigger extends Trigger {

  private Trigger primaryTrigger;
  private Trigger disableTrigger;

  /**
   * Create a new ButtonNotTrigger
   * 
   * @param primaryTrigger The trigger that the output is based on
   * @param disableTrigger The trigger that disables the primary trigger
   */
  public ButtonNotTrigger(Trigger primaryTrigger, Trigger disableTrigger) {
    this.primaryTrigger = primaryTrigger;
    this.disableTrigger = disableTrigger;
  }

  @Override
  public boolean get() {
    return primaryTrigger.get() && !disableTrigger.get();
  }
}
