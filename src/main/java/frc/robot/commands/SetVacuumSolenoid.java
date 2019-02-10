/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Vacuum.VacSolenoid;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class SetVacuumSolenoid extends InstantCommand {
  /**
   * Add your docs here.
   */
  public SetVacuumSolenoid(VacSolenoid id, boolean isOn) {
    super();
    requires(Robot.vacuum);
    Robot.vacuum.setSolenoid(id, isOn);
    System.out.println("Hooray " + id + "'s state " + isOn);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
  }

}
