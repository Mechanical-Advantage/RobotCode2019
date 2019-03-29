/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Locks the beaver tail release pistons
 */
public class LockBeaverTail extends Command {

  public LockBeaverTail() {
    super();
    requires(Robot.beaverTail);
    this.setInterruptible(false); // Block commands that want the beaver tail from initializing
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.beaverTail.lockTail();
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}
