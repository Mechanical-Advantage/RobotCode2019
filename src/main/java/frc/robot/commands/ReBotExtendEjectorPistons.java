/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 * Extends the hatch ejector pistons briefly
 */
public class ReBotExtendEjectorPistons extends TimedCommand {
  public ReBotExtendEjectorPistons(double timeout) {
    super(timeout);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    // requires(Robot.reBotIntake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Robot.reBotIntake.extendEjectorPistons();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Called once after timeout
  @Override
  protected void end() {
    // Robot.reBotIntake.retractEjectorPistons();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    // Robot.reBotIntake.retractEjectorPistons();
  }
}