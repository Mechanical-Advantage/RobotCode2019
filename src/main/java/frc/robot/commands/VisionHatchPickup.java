/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.LatencyData;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.RobotType;
import frc.robot.subsystems.DriveTrain.DriveGear;

public class VisionHatchPickup extends Command {

  private static final int dataPoints = 100;
  private static final double kUpdatePeriodDistance = 0.02;
  private static final double kUpdatePeriodAngle = 0.05;
  private static final double kTurnCorrectionAmount = 0.2;
  // PID output will be limited to negative to positive this
  private static final double kMaxOutput = 0.9;

  private double kPDistance;
  private double kIDistance;
  private double kDDistance;
  private double kFDistance;
  private double kPAngle;
	private double kIAngle;
	private double kDAngle;
  private double kFAngle;
  private DriveGear gear;

  private LatencyData distanceData = new LatencyData(dataPoints);
  private LatencyData angleData = new LatencyData(dataPoints);
  private float previousYaw;
  private double previousDistance;
  private PIDController distanceController, turnController;
  private DriveOutputter driveOutputter = new DriveOutputter();
  private boolean visionDataRecieved; // Whether any vision data has been recieved

  public VisionHatchPickup() {
    super();
    requires(Robot.visionData);
    requires(Robot.driveSubsystem);

    switch (RobotMap.robot) {
      case ROBOT_2017:
        kPDistance = 0.0032;
        kIDistance = 0.000000;
        kDDistance = 0;
        kFDistance = 0;
        kPAngle = 0.015;
        kIAngle = 0;
        kDAngle = 0;
        kFAngle = 0;
        break;
      case ORIGINAL_ROBOT_2018:
        kPDistance = 0.02;
        kIDistance = 0.000000;
        kDDistance = 0;
        kFDistance = 0;
        kPAngle = 0.05; // was 0.05, disabled due to WPILib Tolerance buffer phase lag
        kIAngle = 0;
        kDAngle = 0;
        kFAngle = 0;
        gear = DriveGear.HIGH;
        break;
      case EVERYBOT_2019:
        kPDistance = 0.017;
        kIDistance = 0;
        kDDistance = 0;
        kFDistance = 0.5;
        kPAngle = 0.07;
        kIAngle = 0;
        kDAngle = 0;
        kFAngle = 0;
        break;
      case ROBOT_2019:
      case ROBOT_2019_2:
        kPDistance = 0.;
        kIDistance = 0.000000;
        kDDistance = 0;
        kFDistance = 0;
        kPAngle = 0;
        kIAngle = 0;
        kDAngle = 0;
        kFAngle = 0;
        break;
      default:
        break;
    }
    distanceController = new PIDController(kPDistance, kIDistance, kDDistance, kFDistance, distanceData, driveOutputter, kUpdatePeriodDistance);
		turnController = new PIDController(kPAngle, kIAngle, kDAngle, kFAngle, angleData, driveOutputter.getAngleReciever(), kUpdatePeriodAngle);
		distanceController.setOutputRange(-1, 1);
		turnController.setOutputRange(-1, 1);
		turnController.setInputRange(-180, 180); // should this be field of view?
		turnController.setContinuous();
		distanceController.setSetpoint(0);
		turnController.setSetpoint(0);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    visionDataRecieved = false;
    if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			Robot.driveSubsystem.switchGear(gear);
		}
    distanceData.clear();
    angleData.clear();
    Robot.visionData.setPipeline(Robot.visionData.hatch);
    previousYaw = Robot.ahrs.getYaw(); // This makes the initial difference 0
    previousDistance = (Robot.driveSubsystem.getDistanceLeft() +
    Robot.driveSubsystem.getDistanceRight()) / 2;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double currentRawDistance = (Robot.driveSubsystem.getDistanceLeft() +
    Robot.driveSubsystem.getDistanceRight()) / 2;
    // Subtract because encoder values go in the opposite direction thsn vision 
    distanceData.addDataPoint(distanceData.getCurrentPoint() - (currentRawDistance - previousDistance));
    previousDistance = currentRawDistance;
    float currentRawYaw = Robot.ahrs.getYaw();
    // Calculate how much the gyro reading changed and apply that to the last data point
    // Subtract because gyro positive is backwards from angle needed to reach target
    angleData.addDataPoint(angleData.getCurrentPoint() - (currentRawYaw - previousYaw));
    previousYaw = currentRawYaw;
    if (!Robot.visionData.hatch.isDataHandled()) {
      boolean dataApplied = distanceData.addCorrectedData(
        Robot.visionData.hatch.getDistance(), 
        Robot.visionData.hatch.getLastTimestamp());
      dataApplied = dataApplied && angleData.addCorrectedData(
        Robot.visionData.hatch.getAngle(), 
        Robot.visionData.hatch.getLastTimestamp());
      Robot.visionData.hatch.dataHandled();
      // Start actually driving if this is the first data and
      // vision data has been incorporated into the LatencyData objs
      if (!visionDataRecieved && dataApplied) {
        turnController.enable();
        distanceController.enable();    
      }
      visionDataRecieved = true;
    }
    // System.out.println("Distance: " + distanceData.getCurrentPoint());
    // System.out.println("Angle: " + angleData.getCurrentPoint());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    turnController.disable();
    distanceController.disable();
    Robot.driveSubsystem.stop();
    Robot.visionData.stopPipeline();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

  private static class DriveOutputter implements PIDOutput {

    private double angleOutput;
    private AngleReciever angleReciever = new AngleReciever();

    @Override
    public void pidWrite(double output) {
      // Inverted velocity because PID is trying to push input lower
		  double outputVelocity = output*(kMaxOutput-kTurnCorrectionAmount)*-1;
		  double outputTurnVelocity = angleOutput*kTurnCorrectionAmount;
		  Robot.driveSubsystem.drive(outputVelocity-outputTurnVelocity, outputVelocity+outputTurnVelocity);
    }

    private AngleReciever getAngleReciever() {
      return angleReciever;
    }

    private class AngleReciever implements PIDOutput {
      @Override
      public void pidWrite(double output) {
        angleOutput = output;
      }
    }
  }
}
