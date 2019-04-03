/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class EjectCargo extends CommandGroup {

  private static final double inTime = 0.75;
  private static final double inPower = 0.15;

  /**
   * Pulls in and then ejects the cargo
   */
  public EjectCargo() {
    addSequential(new RunArmLightIntake(inPower), inTime);
    addSequential(new RunArmLightIntake(true));
  }
}
