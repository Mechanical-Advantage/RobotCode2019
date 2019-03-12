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
import frc.robot.subsystems.Vacuum.VacSolenoid;

public class VacPickup extends Command {

  private static final double suctionGoodThreshold = 0.55;

  private boolean LEDOnLast;

  public VacPickup() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    super();
    requires(Robot.vacuum);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.oi.updateLED(OILED.VAC_PICKUP, true);
    Robot.vacuum.setVacuumMotor(true);
    Robot.vacuum.setSolenoid(VacSolenoid.PICKUP, true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.vacuum.getPressureSensorVoltage() > suctionGoodThreshold) {
      if (!LEDOnLast) {
        Robot.oi.updateLED(OILED.MISC_3, true);
        LEDOnLast = true;
      }
    } else {
      if (LEDOnLast) {
        Robot.oi.updateLED(OILED.MISC_3, false);
        LEDOnLast = false;
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
    Robot.vacuum.setSolenoid(VacSolenoid.PICKUP, false);
    Robot.vacuum.setVacuumMotor(false);
    Robot.oi.updateLED(OILED.VAC_PICKUP, false);
    Robot.oi.updateLED(OILED.MISC_3, false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
