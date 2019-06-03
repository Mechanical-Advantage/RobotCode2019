/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.Vacuum.VacuumLevel;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ReBotEnableVacuum extends Command {

  private static final double suctionGoodThreshold = 0.55; // For SmartDashboard indicator

  private boolean indicatorOnLast;

  public ReBotEnableVacuum() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    requires(Robot.vacuum);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.vacuum.setVacuumMotor(VacuumLevel.HIGH);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double pressureSensorVoltage = Robot.vacuum.getPressureSensorVoltage();
    if (pressureSensorVoltage > suctionGoodThreshold) {
      if (!indicatorOnLast) {
        SmartDashboard.putBoolean("Suction Good", true);
        indicatorOnLast = true;
      }
    } else {
      if (indicatorOnLast) {
        SmartDashboard.putBoolean("Suction Good", false);
        indicatorOnLast = false;
      }
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
    SmartDashboard.putBoolean("Suction Good", false);
    indicatorOnLast = false;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
