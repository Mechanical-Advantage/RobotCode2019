/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.RobotType;

/**
 * Add your docs here.
 */
public class ToggleIntakeRetracted extends TimedCommand {

  private static final double runTime = 1;

  public ToggleIntakeRetracted() {
    super(runTime);
    requires(Robot.intake);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      if (Robot.intake.getRetractedLeft() && Robot.intake.getRetractedRight()) {
        Robot.intake.intake();
        Robot.intake.setRetracted(false);
      } else {
        Robot.intake.setRetracted(true);
      }
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Called once after timeout
  @Override
  protected void end() {
    Robot.intake.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
