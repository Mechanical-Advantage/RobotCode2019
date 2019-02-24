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
import frc.robot.subsystems.Arm;

public class ArmTuning extends Command {
  public ArmTuning() {
    super();
    requires(Robot.arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putNumber("Arm Elbow Position", Robot.arm.getElbowPosition());
    SmartDashboard.putNumber("Arm Wrist Position", Robot.arm.getWristPosition());
    SmartDashboard.putNumber("Arm Telescope Position", Robot.arm.getTelescopePosition());
    // Robot.arm.setShoulderRaised(SmartDashboard.getBoolean("Arm Shoulder High", false));
    
    if (SmartDashboard.getBoolean("Arm Elbow/enabled", false)) {
      Robot.arm.setElbowPosition(SmartDashboard.getNumber("Arm Elbow/setpoint", 0.0));
      Robot.arm.enableElbow();
    } else if (SmartDashboard.getBoolean("Arm Elbow Open Loop/enabled", false)) {
      Robot.arm.runElbowOpenLoop(SmartDashboard.getNumber("Arm Elbow Open Loop/speed", 0.0));
      Robot.arm.enableElbow();
    } else {
      Robot.arm.disableElbow();
    }

    if (SmartDashboard.getBoolean("Arm Wrist/enabled", false)) {
      Robot.arm.setWristPosition(Arm.WristPosition.values()[(int)SmartDashboard.getNumber("Arm Wrist/setpoint", 0)]);
      Robot.arm.enableWrist();
    } else {
      Robot.arm.disableWrist();
    }

    if (SmartDashboard.getBoolean("Arm Telescope/enabled", false)) {
      Robot.arm.setTelescopePosition(SmartDashboard.getNumber("Arm Telescope/setpoint", 0.0));
      Robot.arm.enableTelescope();
    } else {
      Robot.arm.disableTelescope();
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
