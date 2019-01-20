/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Cancels the passed command
 */
public class CancelCommand extends InstantCommand {

  private Command command;
  
  public CancelCommand(Command command) {
    super();
    this.command = command;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    command.cancel();
  }

}
