/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.triggers;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.buttons.Trigger;

/**
 * Switches between buttons based on whether demo controls are active
 */
public class DemoSwitchingTrigger extends Trigger {

  private Trigger standardTrigger;
  private Trigger demoTrigger;

  /**
   * @param standardTrigger Trigger (button) to use when not in demo mode
   * @param demoTrigger     Trigger (button) to use when in demo mode
   */
  public DemoSwitchingTrigger(Trigger standardTrigger, Trigger demoTrigger) {
    this.standardTrigger = standardTrigger;
    this.demoTrigger = demoTrigger;
  }

  @Override
  public boolean get() {
    Trigger activeTrigger;
    if (Robot.oi.getDemoMode()) {
      activeTrigger = demoTrigger;
    } else {
      activeTrigger = standardTrigger;
    }
    if (activeTrigger == null) {
      return false;
    } else {
      return activeTrigger.get();
    }
  }
}
