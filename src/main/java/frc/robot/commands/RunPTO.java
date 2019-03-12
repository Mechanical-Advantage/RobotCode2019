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

public class RunPTO extends Command {

  // Whether to invert the direction of travel (relative to standard drive)
  private static final boolean invert = true;
  // Whether to disable brake mode while running this command
  private static final boolean disableBrakeMode = true;

  private boolean canRun;

  public RunPTO() {
    super();
    requires(Robot.driveSubsystem);
    requires(Robot.simpleScorer);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (Robot.beaverTail.getTailLocked()) {
      canRun = false;
    } else {
      Robot.simpleScorer.retract();
      Robot.driveSubsystem.enablePTO();
      if (disableBrakeMode) {
        Robot.driveSubsystem.enableBrakeMode(false);
      }
      canRun = true;
      Robot.oi.updateLED(OILED.MISC_1, true);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (canRun) {
      // Joysticks are flipped in DriveWIthJoystick so inverting works backwards to be consistent
      Robot.driveSubsystem.runPTO(Robot.oi.getSingleDriveAxis() * (invert ? 1 : -1));
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !canRun;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveSubsystem.disablePTO();
    if (disableBrakeMode) {
      Robot.driveSubsystem.enableBrakeMode(true);
    }
    Robot.oi.updateLED(OILED.MISC_1, false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
