package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Drives with the joystick from OI
 */
public class DriveWithJoystick extends Command {

  private static final boolean alwaysUseHighMaxVel = true; // Whether to always use the max velocity of high gear or
                                                           // of current gear
  private static final double rightStickScale = 0.5; // Factor of right stick when added

  public DriveWithJoystick() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    super("DriveWithJoystick");
    requires(Robot.driveSubsystem);
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    SmartDashboard.putBoolean("Driver Control", true);
  }

  private double processJoystickAxis(double joystickAxis) {
    // cube to improve low speed control, multiply by -1 because negative joystick
    // means forward, 0 if within deadband
    return Math.abs(joystickAxis) > Robot.oi.getDeadband() ? joystickAxis * Math.abs(joystickAxis) * -1 : 0;
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
    double joystickLeft = 0, joystickRight = 0;
    double baseDrive;
    double totalTurn;
    switch (Robot.joystickModeChooser.getSelected()) {
    case Tank:
      joystickRight = processJoystickAxis(Robot.oi.getRightAxis());
      joystickLeft = processJoystickAxis(Robot.oi.getLeftAxis());
      break;
    case SplitArcade:
      baseDrive = processJoystickAxis(Robot.oi.getSingleDriveAxisLeft());
      joystickRight = baseDrive + processJoystickAxis(Robot.oi.getRightHorizDriveAxis());
      joystickRight = joystickRight > 1 ? 1 : joystickRight;
      joystickLeft = baseDrive - processJoystickAxis(Robot.oi.getRightHorizDriveAxis());
      joystickLeft = joystickLeft > 1 ? 1 : joystickLeft;
      break;
    case SplitArcadeRightDrive:
      baseDrive = processJoystickAxis(Robot.oi.getSingleDriveAxisRight());
      joystickRight = baseDrive + processJoystickAxis(Robot.oi.getLeftHorizDriveAxis());
      joystickRight = joystickRight > 1 ? 1 : joystickRight;
      joystickLeft = baseDrive - processJoystickAxis(Robot.oi.getLeftHorizDriveAxis());
      joystickLeft = joystickLeft > 1 ? 1 : joystickLeft;
      break;
    case Trigger:
      baseDrive = (Robot.oi.getLeftTrigger() - Robot.oi.getRightTrigger()) * -1;
      totalTurn = Robot.oi.getLeftHorizDriveAxis() + (Robot.oi.getRightHorizDriveAxis() * rightStickScale);
      joystickRight = baseDrive + processJoystickAxis(totalTurn);
      joystickRight = joystickRight > 1 ? 1 : joystickRight;
      joystickLeft = baseDrive - processJoystickAxis(totalTurn);
      joystickLeft = joystickLeft > 1 ? 1 : joystickLeft;
      break;
    }
    Robot.driveSubsystem.drive(joystickLeft, joystickRight, alwaysUseHighMaxVel);
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    SmartDashboard.putBoolean("Driver Control", false);
  }

  public static enum JoystickMode {
    Tank, SplitArcade, SplitArcadeRightDrive, Trigger;
  }
}
