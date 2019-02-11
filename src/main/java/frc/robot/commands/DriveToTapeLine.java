/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.LatencyData;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.RobotType;
import frc.robot.subsystems.DriveTrain.DriveGear;
import jaci.pathfinder.Pathfinder;

public class DriveToTapeLine extends Command {

  private static final int dataPoints = 100;
  private static final double kUpdatePeriodDistance = 0.02;
  private static final double kUpdatePeriodAngle = 0.05;
  // % velocity reserved for turn correction
  private static final double kTurnCorrectionAmount = 0.1;
  // PID output will be limited to negative to positive this
  private static final double kMaxOutput = 0.3;
  // Limit change in one iteration to this - % of max output
  private static final double kMaxChange = 0.06;
  private static final double targetDistance = 12; // Distance from camera to try to get to
  private static final double distanceTolerance = 0.5;
  private static final double angleTolerance = 1;
  private static final float tapeAngle = -90; // Varies based on which tape line
  private static final double maxTargetY = 36; // The maximum distance away the angle will target based on target distance
  private static final double maxTargetYDistance = 48; // What distance from target should be maxTargetY as a target, scales down at closer distances

  private double kPDistance;
  private double kIDistance;
  private double kDDistance;
  private double kFDistance;
  private double kPAngle;
  private double kIAngle;
  private double kDAngle;
  private double kFAngle;
  private DriveGear gear;

  private static double maxOutputVelocityChange = kMaxOutput * kMaxChange;
  private LatencyData xData = new LatencyData(dataPoints);
  private LatencyData yData = new LatencyData(dataPoints);
  private LatencyData angleData = new LatencyData(dataPoints); // This is not used for correction but just to get historical angle data
  private float previousYaw;
  private double previousDistance;
  private PIDController distanceController, turnController;
  private DriveOutputter driveOutputter = new DriveOutputter();
  private DistanceCalculator distanceCalc = new DistanceCalculator();
  private boolean visionDataRecieved; // Whether any vision data has been recieved

  /*
  Coordinate System:
  Origin=target
  Y=Perpendicular to target plane, positive = further away
  X=Parallel to target plane, positive = further right
  
  A point along the y axis is picked as a target and the angle controller is aimed at it.
  How far out is based on distance to target
  */

  public DriveToTapeLine() {
    super();
    requires(Robot.driveSubsystem);
    requires(Robot.visionData);

    switch (RobotMap.robot) {
    case ROBOT_2017:
      kPDistance = 0.04; // 0.0032 slow but good
      kIDistance = 0.000000;
      kDDistance = 0;
      kFDistance = 0;
      kPAngle = 0.03; // 0.015 works well with 0.0032, slow but good
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
    distanceController = new PIDController(kPDistance, kIDistance, kDDistance, kFDistance, distanceCalc, driveOutputter,
        kUpdatePeriodDistance);
    turnController = new PIDController(kPAngle, kIAngle, kDAngle, kFAngle, angleData, driveOutputter.getAngleReciever(),
        kUpdatePeriodAngle);
    distanceController.setOutputRange(-1, 1);
    turnController.setOutputRange(-1, 1);
    turnController.setInputRange(-180, 180); // should this be field of view?
    turnController.setContinuous();
    distanceController.setSetpoint(targetDistance);
    distanceController.setAbsoluteTolerance(distanceTolerance);
    turnController.setSetpoint(0);
    turnController.setAbsoluteTolerance(angleTolerance);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    visionDataRecieved = false;
    if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
      Robot.driveSubsystem.switchGear(gear);
    }
    xData.clear();
    yData.clear();
    angleData.clear();
    Robot.visionData.setPipeline(Robot.visionData.whiteTape);
    previousYaw = Robot.ahrs.getYaw() - tapeAngle; // This makes the initial difference 0
    previousDistance = (Robot.driveSubsystem.getDistanceLeft() + Robot.driveSubsystem.getDistanceRight()) / 2;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double currentRawDistance = (Robot.driveSubsystem.getDistanceLeft() + Robot.driveSubsystem.getDistanceRight()) / 2;
    float currentYaw = (float)Pathfinder.boundHalfDegrees(Robot.ahrs.getYaw() - tapeAngle);
    // Apply distance moved in average angle
    updateXYData(currentRawDistance - previousDistance, (currentYaw + previousYaw) / 2);
    angleData.addDataPoint(currentYaw);
    previousDistance = currentRawDistance;
    previousYaw = currentYaw;
    if (!Robot.visionData.whiteTape.isDataHandled()) {
      // Calculate x and y
      // Uses the gyro angle at the time the frame was taken in these calculations
      Double angleAtCapture = angleData.getPoint(Robot.visionData.whiteTape.getLastTimestamp());
      if (angleAtCapture != null) {
        double angleToTarget = angleAtCapture
            + Robot.visionData.whiteTape.getAngle();
        double distance = Robot.visionData.whiteTape.getDistance();
        double x = Math.sin(Math.toRadians(angleToTarget)) * distance;
        double y = Math.cos(Math.toRadians(angleToTarget)) * distance;

        boolean dataApplied = xData.addCorrectedData(x, Robot.visionData.whiteTape.getLastTimestamp());
        dataApplied = dataApplied && yData.addCorrectedData(y, Robot.visionData.whiteTape.getLastTimestamp());
        Robot.visionData.whiteTape.dataHandled();
        // Start actually driving if this is the first data and
        // vision data has been incorporated into the LatencyData objs
        if (!visionDataRecieved && dataApplied) {
          turnController.enable();
          distanceController.enable();
        }
        visionDataRecieved = true;
      }
    }
    // Calculate a new angle setpoint
    turnController.setSetpoint(calcAngleSetpoint());
    System.out.println("X: " + xData.getCurrentPoint());
    System.out.println("Y: " + yData.getCurrentPoint());
    System.out.println("Cur angle: " + currentYaw);
    System.out.println("Target angle: " + turnController.getSetpoint());
    System.out.println("Distance: " + distanceCalc.pidGet());
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
    turnController.reset();
    distanceController.disable();
    distanceController.reset();
    Robot.driveSubsystem.stop();
    Robot.visionData.stopPipeline();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

  private double calcAngleSetpoint() {
    double distance = distanceCalc.pidGet();
    double targetY;
    if (distance >= maxTargetYDistance) {
      // map will scale out of range values so this enforces the maximum
      targetY = maxTargetY;
    } else {
      targetY = Robot.map(distance, 0, maxTargetYDistance, 0, maxTargetY);
    }
    double angle = Math.atan2(yData.getCurrentPoint()-targetY, 
      xData.getCurrentPoint());
    angle = Math.toDegrees(angle);
    // Converts to coordinate system values
    return Pathfinder.boundHalfDegrees((90-angle)*-1);
  }

  private void updateXYData(double distance, double angle) {
    double dx = Math.sin(Math.toRadians(angle)) * distance;
    // Invert y because driving forward decreases
    double dy = Math.cos(Math.toRadians(angle)) * distance * -1;
    xData.addDataPoint(xData.getCurrentPoint() + dx);
    yData.addDataPoint(yData.getCurrentPoint() + dy);
  }

  private class DistanceCalculator implements PIDSource {

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
      // Not supported
    }

    @Override
    public PIDSourceType getPIDSourceType() {
      return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
      // TODO this is not the best algorithm
      // Note: calcAngleSetpoint also uses this function and needs true distance
      // While the distanceController input should really be path distance
      return Math.sqrt(Math.pow(xData.getCurrentPoint(), 2) + 
        Math.pow(yData.getCurrentPoint(), 2));
    }

  }

  private static class DriveOutputter implements PIDOutput {

    private double angleOutput;
    private AngleReciever angleReciever = new AngleReciever();
    private double lastOutput;

    @Override
    public void pidWrite(double output) {
      // Inverted velocity because PID is trying to push input lower
      double outputVelocity = output*(kMaxOutput-kTurnCorrectionAmount)*-1;
      outputVelocity = calcNewVelocity(outputVelocity, lastOutput);
      lastOutput = outputVelocity;
		  double outputTurnVelocity = angleOutput*kTurnCorrectionAmount;
		  Robot.driveSubsystem.drive(outputVelocity+outputTurnVelocity, outputVelocity-outputTurnVelocity);
    }

    private double calcNewVelocity(double currentOutput, double lastOutput) {
    	double targetOutput = currentOutput*(kMaxOutput-kTurnCorrectionAmount);
    	if (Math.abs(lastOutput - targetOutput) > maxOutputVelocityChange){
    		if (lastOutput < targetOutput) {
        		targetOutput = lastOutput + maxOutputVelocityChange;
    		} else {
    			targetOutput = lastOutput - maxOutputVelocityChange;
    		}
    	}
    	return targetOutput;
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
