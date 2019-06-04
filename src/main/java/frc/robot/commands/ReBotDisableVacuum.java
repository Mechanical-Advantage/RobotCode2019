/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.Vacuum.VacuumLevel;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Disables vacuum motor
 */
public class ReBotDisableVacuum extends InstantCommand {

  public ReBotDisableVacuum() {
    super("ReBotDisableVacuum");
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    requires(Robot.vacuum);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.vacuum.setVacuumMotor(VacuumLevel.OFF);
  }

}
