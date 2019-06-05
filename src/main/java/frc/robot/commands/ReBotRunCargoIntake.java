/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Intake.GamePiece;;

public class ReBotRunCargoIntake extends Command {

  private final Double intakeSpeed = (double) -1;
  private final Double ejectSpeed = (double) 1;

  private Double speed;

  public ReBotRunCargoIntake(IntakeAction action) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    requires(Robot.intake);
    switch (action) {
    case INTAKE:
      speed = intakeSpeed;
      Robot.intake.setGamepiece(GamePiece.CARGO);
      break;
    case EJECT:
      speed = ejectSpeed;
      break;
    }
  }

  public ReBotRunCargoIntake(Double speed) {
    requires(Robot.intake);
    this.speed = speed;
    if ((speed > 1 && intakeSpeed > 1) || (speed < 1 && intakeSpeed < 1)) {
      Robot.intake.setGamepiece(GamePiece.CARGO);
    }
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.intake.run(speed);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
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
    Robot.intake.stop();
  }

  public enum IntakeAction {
    INTAKE, EJECT
  }
}
