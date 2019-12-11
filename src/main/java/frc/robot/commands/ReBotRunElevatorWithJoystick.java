/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class ReBotRunElevatorWithJoystick extends Command {

  private final double deadband = 0.05;
  private final double maxVelocity = 2; // inches/second
  private final boolean openLoopDrive = true; // if true use percent output, if false move setpoint. if move to setpoint
                                              // in open loop, joystick driving in closed loop will not work after
                                              // moving
  private final boolean openLoopHold = true; // if true use neutral mode, if false use position closed loop

  private static boolean movingLast;

  public ReBotRunElevatorWithJoystick() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    super("ReBotRunElevatorWithJoystick");
    requires(Robot.elevator);
    movingLast = false;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double joystickAxis = Robot.oi.getElevatorStick();
    joystickAxis = Math.abs(joystickAxis) > deadband ? joystickAxis * Math.abs(joystickAxis) * -1 : 0;
    if (joystickAxis == 0) {
      if (movingLast) {
        movingLast = false;
        if (openLoopHold) {
          Robot.elevator.stop();
        } else {
          Robot.elevator.moveToPosition(Robot.elevator.getElevatorPosition());
        }
      }
    } else {
      movingLast = true;
      if (openLoopDrive) {
        Robot.elevator.run(joystickAxis);
      } else {
        Robot.elevator.moveToPosition(Robot.elevator.getTargetPosition() + (maxVelocity / 50 * joystickAxis)); // convert
                                                                                                               // max
                                                                                                               // velocity
                                                                                                               // to
                                                                                                               // inches
                                                                                                               // per
                                                                                                               // controller
        // cycle
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
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.elevator.stop();
  }
}
