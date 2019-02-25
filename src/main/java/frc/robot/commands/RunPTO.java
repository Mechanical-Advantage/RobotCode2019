/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class RunPTO extends Command {

  // Whether to invert the direction of travel (relative to standard drive)
  private static final boolean invert = true;
  // Whether to disable brake mode while running this command
  private static final boolean disableBrakeMode = true;

  public RunPTO() {
    super();
    requires(Robot.driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveSubsystem.enablePTO();
    if (disableBrakeMode) {
      Robot.driveSubsystem.enableBrakeMode(false);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Joysticks are flipped in DriveWIthJoystick to inverting works backwards to be consistent
    Robot.driveSubsystem.runPTO(Robot.oi.getSingleDriveAxis() * (invert ? 1 : -1));
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
    if (disableBrakeMode) {
      Robot.driveSubsystem.enableBrakeMode(true);
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
