/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.OI.OILED;
import frc.robot.subsystems.Vacuum.VacSolenoid;

public class VacTail extends Command {
  public VacTail() {
    super();
    requires(Robot.vacuum);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.oi.updateLED(OILED.VAC_TAIL, true);
    Robot.vacuum.setSolenoid(VacSolenoid.PUMP_TAIL, true);
    Robot.vacuum.setVacuumMotor(true);
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
    Robot.vacuum.setVacuumMotor(false);
    Robot.oi.updateLED(OILED.VAC_TAIL, true);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
