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
 * Retracts the simple scorer
 */
public class RetractSimpleScorer extends InstantCommand {

  public RetractSimpleScorer() {
    super();
    requires(Robot.simpleScorer);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.simpleScorer.retract();
  }
}
