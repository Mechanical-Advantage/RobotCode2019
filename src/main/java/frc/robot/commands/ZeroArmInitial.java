/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ZeroArmInitial extends Command {

  private ZeroState state;

  public ZeroArmInitial() {
    super();
    requires(Robot.arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.arm.zeroTelescope();
    state = ZeroState.TELESCOPE;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    switch (state) {
      case TELESCOPE:
        if (Robot.arm.isTelescopeZeroed()) {
          Robot.arm.zeroWrist();
          state = ZeroState.WRIST;
        }
        break;
      case WRIST:
        if (Robot.arm.isWristZeroed()) {
          Robot.arm.beginElbowZeroSequence();
          state = ZeroState.FINISHED;
        }
        break;
      default:
        break;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return state == ZeroState.FINISHED;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    // Subsystem should deal with this properly
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.arm.disableElbow();
    Robot.arm.disableWrist();
    Robot.arm.disableTelescope();
  }

  private enum ZeroState {
    TELESCOPE, WRIST, FINISHED;
  }
}
