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

public class LowerPTO extends Command {

  private static final double rotations = -12.45;

  public LowerPTO() {
    super();
    requires(Robot.driveSubsystem);
    requires(Robot.beaverTail);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.beaverTail.releaseTail();
    Robot.driveSubsystem.enablePTO();
    Robot.driveSubsystem.drivePTOToPosition(rotations);
    Robot.oi.updateLED(OILED.MISC_1, true);
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
    Robot.driveSubsystem.disablePTO();
    Robot.oi.updateLED(OILED.MISC_1, false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
