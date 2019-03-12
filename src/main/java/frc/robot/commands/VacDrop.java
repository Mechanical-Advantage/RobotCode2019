/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Vacuum.VacSolenoid;

public class VacDrop extends TimedCommand {

  private static final double time = 5; // sec

  public VacDrop() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    super(time);
    requires(Robot.vacuum);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.vacuum.setVacuumMotor(false); // This shouldn't do anything but just make sure
    Robot.vacuum.setSolenoid(VacSolenoid.PICKUP, true);
    Robot.vacuum.setSolenoid(VacSolenoid.PUMP_TAIL, true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.vacuum.setSolenoid(VacSolenoid.PICKUP, false);
    Robot.vacuum.setSolenoid(VacSolenoid.PUMP_TAIL, false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
