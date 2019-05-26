package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Run the intake while the command is running
 */
public class RunSimpleScorerIntake extends Command {

  private double speed;
  private boolean eject;

  /**
   * Create a new RunSimpleScorerIntake using slider power
   * 
   * @param eject Whether to eject the cargo
   */
  public RunSimpleScorerIntake(boolean eject) {
    super();
    requires(Robot.simpleScorer);
    this.eject = eject;
  }

  /**
   * Create a new RunSimpleScorerIntake at a fixed power
   * 
   * @param speed Speed to run at (-negative for eject)
   */
  public RunSimpleScorerIntake(double speed) {
    super();
    requires(Robot.simpleScorer);
    this.speed = speed;
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    if (speed == 0) {
      Robot.simpleScorer.runIntake(Robot.oi.getSliderLevel() * (eject ? -1 : 1));
    } else {
      Robot.simpleScorer.runIntake(speed);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  protected void end() {
    Robot.simpleScorer.stopIntake();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    end();
  }
}
