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
 * Toggles between vac pickup and drop
 */
public class VacPickupToggle extends InstantCommand {

  private boolean dropNext = false;
  private final Command pickupCommand = new VacPickup();
  private final Command dropCommand = new VacDrop();

  public VacPickupToggle() {
    super();
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (dropNext) {
      dropCommand.start();
      dropNext = false;
    } else {
      pickupCommand.start();
      dropNext = true;
    }
  }
}
