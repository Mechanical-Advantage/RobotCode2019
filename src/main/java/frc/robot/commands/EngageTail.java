/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Vacuum.VacSolenoid;

public class EngageTail extends Command {
  public EngageTail() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    Robot.vacuum.setVacuumMotor(true);
    Robot.vacuum.setSolenoid(VacSolenoid.PUMP_TAIL, true);
    Robot.vacuum.setSolenoid(VacSolenoid.TAIL_TANK, true);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    end();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.vacuum.setSolenoid(VacSolenoid.PUMP_TAIL, false);
    Robot.vacuum.setSolenoid(VacSolenoid.TAIL_TANK, false);
    Robot.vacuum.setVacuumMotor(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
