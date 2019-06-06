/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Intake.GamePiece;
import frc.robot.subsystems.Elevator;

public class ReBotMoveToSetpoint extends Command {

  private final boolean openLoop = true;
  private final double velocity = 0.3; // when using open loop drive

  private double target;
  private OIElevatorPosition OIPosition;
  private ElevatorPosition position;

  public ReBotMoveToSetpoint(OIElevatorPosition position) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    super();
    requires(Robot.elevator);
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
      if (Robot.intake.getGamepiece() == GamePiece.CARGO) {
        position = ElevatorPosition.CARGO_L1_SHIP;
      } else {
        position = ElevatorPosition.HATCHES_L1;
      }
      break;
    case ROCKET_L1:
      if (Robot.intake.getGamepiece() == GamePiece.CARGO) {
        position = ElevatorPosition.CARGO_L1_ROCKET;
      } else {
        position = ElevatorPosition.HATCHES_L1;
      }
      break;
    case ROCKET_L2:
      if (Robot.intake.getGamepiece() == GamePiece.CARGO) {
        position = ElevatorPosition.CARGO_L2;
      } else {
        position = ElevatorPosition.HATCHES_L2;
      }
      break;
    }

    target = position.getSetpointValue();
    if (openLoop) {
      Robot.elevator
          .run((Robot.elevator.getElevatorPosition() - target) > 0 ? Math.abs(velocity) : Math.abs(velocity) * -1);
    } else {
      Robot.elevator.moveToPosition(target);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Math.abs(Robot.elevator.getElevatorPosition() - target) < Elevator.allowableError;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    if (openLoop) {
      Robot.elevator.stop();
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    if (openLoop) {
      Robot.elevator.stop();
    } else {
      Robot.elevator.moveToPosition(Robot.elevator.getElevatorPosition());
    }
  }

  private enum ElevatorPosition {
    FLOOR, HATCHES_L1, HATCHES_L2, CARGO_L1_SHIP, CARGO_L1_ROCKET, CARGO_L2;

    private double getSetpointValue() {
      switch (this) {
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
  }

  public enum OIElevatorPosition {
    FLOOR, SHIP, ROCKET_L1, ROCKET_L2;
  }
}
