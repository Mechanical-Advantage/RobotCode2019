package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Run the intake while the command is running
 */
public class RunArmLightIntake extends Command {
	
	private double speed;
	private boolean eject;
	
	/**
	 * Create a new EjectCargo using slider power
	 * 
	 * @param eject Whether to eject the cargo
	 */
	public RunArmLightIntake(boolean eject) {
		super();
		requires(Robot.intake);
		this.eject = eject;
	}

	/**
	 * Create a new RunArmLightIntake at a fixed power
	 * 
	 * @param speed Speed to run at (-negative for eject)
	 */
	public RunArmLightIntake(double speed) {
		super();
		requires(Robot.intake);
		this.speed = speed;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		if (speed == 0) {
			Robot.intake.run(Robot.oi.getSliderLevel()* (eject ? -1 : 1));
		} else {
			Robot.intake.run(speed);
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
		Robot.intake.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
