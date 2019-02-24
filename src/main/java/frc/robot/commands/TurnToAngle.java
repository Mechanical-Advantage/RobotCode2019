package frc.robot.commands;

import frc.robot.util.PIDControllerFixed;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.DriveTrain.DriveGear;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Turns the specified number of degrees, -180 to 180. Resets the gyro at the beginning
 */
public class TurnToAngle extends Command implements PIDOutput {
	
	private double kP;
    private double kI;
    private double kD;
    private double kF;
    private double kToleranceDegrees;
    private int kToleranceBufSamples;
    private double updatePeriod;
    private DriveGear gear;
    private PIDControllerFixed turnController;
    private double targetAngle;
    private boolean absoluteAngle;
    
    private double rotateToAngleRate;
    
    public TurnToAngle(double angle, boolean absoluteAngle, double tolerance) {
				this(angle, absoluteAngle);
				kToleranceDegrees = tolerance;
    }
    
    public TurnToAngle(double angle, double tolerance) {
    		this(angle);
    		kToleranceDegrees = tolerance;
    }
    
    public TurnToAngle(double angle) {
    		this(angle, false);
    }

    public TurnToAngle(double angle, boolean absoluteAngle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	super("TurnToAngle");
    	requires(Robot.driveSubsystem);

        // limit input to -180 to 180
        targetAngle = (angle>180) ? 180 : angle;
        targetAngle = (targetAngle<-180) ? -180 : targetAngle;
        switch (RobotMap.robot) {
        	case ROBOT_2017:
	        	kP = 0.0077; // 0.008
	        	kI = 0;
	        	kD = 0.0137; // 0.014
	        	kF = 0;
	        	kToleranceDegrees = 1.0;
	        	kToleranceBufSamples = 10;
	        	updatePeriod = 0.02;
	        	break;
        	case ORIGINAL_ROBOT_2018:
        		kP = 0.007;
        		kI = 0;
        		kD = 0.02;
        		kF = 0;
        		kToleranceDegrees = 1;
        		kToleranceBufSamples = 3;
        		updatePeriod = 0.02;
        		gear = DriveGear.LOW;
        		break;
					case EVERYBOT_2019:
						// Tested on everybot 2018
						kP = 0.01;
						kI = 0;
						kD = 0.003;
						kF = 0;
						kToleranceDegrees = 1.0;
						kToleranceBufSamples = 10;
						updatePeriod = 0.05;
						break;
					case ROBOT_2019:
					case ROBOT_2019_2:
						kP = 0.007;
						kI = 0;
						kD = 0.015;
						kF = 0;
						kToleranceDegrees = 1;
						kToleranceBufSamples = 3;
						updatePeriod = 0.02;
						break;
					default:
						break;
        }
        this.absoluteAngle = absoluteAngle;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
			if (Robot.driveSubsystem.isDualGear()) {
    		Robot.driveSubsystem.switchGear(gear);
    	}
    	turnController = new PIDControllerFixed(kP, kI, kD, kF, Robot.ahrs, this, updatePeriod);
    	turnController.setInputRange(-180.0f,  180.0f);
        turnController.setOutputRange(-1.0, 1.0);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setToleranceBuffer(kToleranceBufSamples);
        turnController.setContinuous(true);
        
        double currentTargetAngle = absoluteAngle ? targetAngle : Robot.ahrs.getYaw()+targetAngle;
        // limit input to -180 to 180
        currentTargetAngle = (currentTargetAngle>180) ? -180+(currentTargetAngle-180) : currentTargetAngle;
        currentTargetAngle = (currentTargetAngle<-180) ? 180+(currentTargetAngle+180) : currentTargetAngle;
        turnController.setSetpoint(currentTargetAngle);
        turnController.enable();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {	    
		double outputVelocity = rotateToAngleRate;
		if (RobotMap.tuningMode) {
			SmartDashboard.putNumber("Angle", Robot.ahrs.getAngle());
			SmartDashboard.putNumber("Rate", Robot.ahrs.getRate());
			SmartDashboard.putNumber("Yaw", Robot.ahrs.getYaw());
			SmartDashboard.putNumber("Turn to angle rate", rotateToAngleRate);
			SmartDashboard.putNumber("Velocity", outputVelocity);
		}
		Robot.driveSubsystem.drive(outputVelocity, outputVelocity*-1);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (Math.abs(Robot.ahrs.getYaw() - turnController.getSetpoint()) < kToleranceDegrees) && turnController.onTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveSubsystem.stop();
    	turnController.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
    
    @Override
    public void pidWrite(double output) {
    	rotateToAngleRate = output;
    }
}
