/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class ReBotMoveToSetpoint extends Command {

  private static final double allowableError = 2;

  private Double target;
  private OIElevatorPosition OIPosition;
  private ElevatorPosition position;

  public ReBotMoveToSetpoint(OIElevatorPosition position) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    // requires(Robot.elevator);
    this.OIPosition = position;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    switch (OIPosition) {
    case FLOOR:
      position = ElevatorPosition.FLOOR;
      break;
    case SHIP:
      if (Robot.gamePiece == Robot.GamePiece.CARGO) {
        position = ElevatorPosition.CARGO_L1_SHIP;
      } else {
        position = ElevatorPosition.HATCHES_L1;
      }
      break;
    case ROCKET_L1:
      if (Robot.gamePiece == Robot.GamePiece.CARGO) {
        position = ElevatorPosition.CARGO_L1_ROCKET;
      } else {
        position = ElevatorPosition.HATCHES_L1;
      }
      break;
    case ROCKET_L2:
      if (Robot.gamePiece == Robot.GamePiece.CARGO) {
        position = ElevatorPosition.CARGO_L2;
      } else {
        position = ElevatorPosition.HATCHES_L2;
      }
      break;
    }

    target = getSetpointValue(position);
    // Robot.elevator.moveToPosition(target);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
    // return abs(Robot.elevator.position - target) < allowableError;
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

  private double getSetpointValue(ElevatorPosition position) {
    switch (position) {
    case FLOOR:
      return 0;
    case HATCHES_L1:
      return 0;
    case HATCHES_L2:
      return 0;
    case CARGO_L1_SHIP:
      return 0;
    case CARGO_L1_ROCKET:
      return 0;
    case CARGO_L2:
      return 0;
    default:
      return 0;
    }
  }

  private enum ElevatorPosition {
    FLOOR, HATCHES_L1, HATCHES_L2, CARGO_L1_SHIP, CARGO_L1_ROCKET, CARGO_L2;
  }

  public enum OIElevatorPosition {
    FLOOR, SHIP, ROCKET_L1, ROCKET_L2;
  }
}
