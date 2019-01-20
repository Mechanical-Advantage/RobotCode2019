/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class VisionRecieverTest extends Command {
  public VisionRecieverTest() {
    super();
    requires(Robot.visionData);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.visionData.setPipeline(Robot.visionData.hatch);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (!Robot.visionData.hatch.isDataHandled()) {
      System.out.println("Dist: " + Robot.visionData.hatch.getDistance());
      System.out.println("Angle: " + Robot.visionData.hatch.getAngle());
      Robot.visionData.hatch.dataHandled();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.visionData.stopPipeline();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
