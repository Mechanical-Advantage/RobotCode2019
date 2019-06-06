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
  private final Double loweredEjectSpeed = (double) 1;
  private final Double raisedEjectSpeed = (double) 0.7;

  private Double speed;
  private IntakeAction action;
  private Boolean preferSpeed;

  public ReBotRunCargoIntake(IntakeAction action) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    requires(Robot.intake);
    this.action = action;
    this.preferSpeed = false;
  }

  public ReBotRunCargoIntake(Double speed) {
    requires(Robot.intake);
    this.speed = speed;
    this.preferSpeed = true;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (preferSpeed) {
      Robot.intake.run(speed);
      // Set gamepiece to cargo if in same direction as eject speed
      if ((speed > 1 && loweredEjectSpeed > 1) || (speed < 1 && loweredEjectSpeed < 1)) {
        Robot.intake.setGamepiece(GamePiece.CARGO);
        this.action = IntakeAction.INTAKE;
      } else {
        this.action = IntakeAction.EJECT;
      }

    } else if (action == IntakeAction.INTAKE) {
      Robot.intake.run(intakeSpeed);
      Robot.intake.setGamepiece(GamePiece.CARGO);

    } else if (Robot.intake.isRaised()) {
      Robot.intake.run(raisedEjectSpeed);

    } else {
      Robot.intake.run(loweredEjectSpeed);
    }

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
    if (Robot.intake.autoHold & this.action == IntakeAction.INTAKE) {
      Robot.intake.hold();
    } else {
      Robot.intake.stop();
    }
  }

  public enum IntakeAction {
    INTAKE, EJECT
  }
}
