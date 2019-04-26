/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Toggles a level 2 climber solenoid
 */
public class ToggleLevel2Solenoid extends InstantCommand {

  private boolean front;

  public ToggleLevel2Solenoid(boolean front) {
    super();
    requires(Robot.level2Climber);
    this.front = front;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (front) {
      Robot.level2Climber.setFrontExtended(!Robot.level2Climber.isFrontExtended());
    } else {
      Robot.level2Climber.setRearExtended(!Robot.level2Climber.isRearExtended());
    }
  }
}
