/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
//import frc.robot.RobotMap;

public class ArmLightTuning extends Command {
  public ArmLightTuning() {
    super();
    requires(Robot.armLight);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putNumber("Arm Elbow Position", Robot.armLight.getElbowPosition());
    Robot.armLight.setShoulderRaised(SmartDashboard.getBoolean("Arm Shoulder High", false));

    if (SmartDashboard.getBoolean("Arm Elbow/enabled", false)) {
      Robot.armLight.setElbowPosition(SmartDashboard.getNumber("Arm Elbow/setpoint", 0.0));
      Robot.armLight.enableElbow();
    } else {
      Robot.armLight.disableElbow();
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
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
