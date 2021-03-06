package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain.DriveGear;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Switches the current gear of the robot
 */
public class SwitchGear extends InstantCommand {

  private DriveGear gear;

  public SwitchGear(DriveGear gear) {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveSubsystem);
    this.gear = gear;
  }

  // Called once when the command executes
  protected void initialize() {
    Robot.driveSubsystem.switchGear(gear);
  }

}
