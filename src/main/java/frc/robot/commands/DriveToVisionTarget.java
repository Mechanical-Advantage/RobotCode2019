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
import frc.robot.subsystems.DriveTrain.DriveGear;
import frc.robot.subsystems.VisionData.DeliveryTargetPipeline;
import frc.robot.subsystems.VisionData.Pipeline;
import frc.robot.subsystems.VisionData.WhiteTapePipeline;
import jaci.pathfinder.Pathfinder;

public class DriveToVisionTarget extends Command {

  private static final int dataPoints = 100;
  private static final double kUpdatePeriodDistance = 0.01; // Originally 0.02
  private static final double kUpdatePeriodAngle = 0.01; // Originally 0.05
  // % velocity reserved for turn correction
  private static final double kTurnCorrectionAmount = 0.2;
  // PID output will be limited to negative to positive this
  private static final double kMaxOutput = 0.9;
  // Limit change in one iteration to this - % of max output
  private static final double kMaxChange = 0.06;
  private static final double targetDistance = 12; // Distance from camera to try to get to
  private static final double distanceTolerance = 0.5;
  private static final double angleTolerance = 1;
  private static final float tapeAngle = -90; // Varies based on which tape line, only used if processor does not provide vision angle
  private static final double maxTargetY = 48; // The maximum distance away the angle will target based on target distance or current position (see useLookAheadNav)
  private static final double minTargetY = 6; // The mimumum distance away the angle will target, applies to both angle calc modes
  private static final double maxTargetYDistance = 60; // What distance from target should be maxTargetY as a target, scales down at closer distances, only used for non-lookahead mode
  private static final double lookAheadDistance = 8; // How many inches in front of the current y position to target, only used for lookahead mode
  private static final boolean useLookAheadNav = true; // Whether to use look ahead distance or scaling based on distance to get target y (see calcAngleSetpoint)
  private static final double minCameraDistance = 0; // Vision data ignored when closer than this
  private static final Pipeline pipeline = Robot.visionData.delivery;
  // pipelineProcessor cannot be static because it can refer to local variables of this class
  private final PipelineProcessor pipelineProcessor = new RetroSolvePnPProcessor();

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
  private LatencyData angleData = new LatencyData(dataPoints); // This is not used for correction but just to get historical angle data if the processor does not provide vision angle values
  private double previousDistance;
  private float previousRawYaw; // Yaw value from the gyro
  private double previousYaw; // Yaw value used for processing, equals previousRawYaw if processor does not provide vision angle
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

  public DriveToVisionTarget() {
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
      kPAngle = 0.05; // was 0.05
      kIAngle = 0;
      kDAngle = 0;
      kFAngle = 0;
      gear = DriveGear.LOW; // Originally HIGH
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
    if (Robot.driveSubsystem.isDualGear()) {
      Robot.driveSubsystem.switchGear(gear);
    }
    xData.clear();
    yData.clear();
    angleData.clear();
    Robot.visionData.setPipeline(pipeline);
    // Makes the initial differences 0
    if (pipelineProcessor.providesVisionAngle()) {
      previousRawYaw = (float)Pathfinder.boundHalfDegrees(Robot.ahrs.getYaw());
      previousYaw = 0;
    } else {
      previousRawYaw = (float)Pathfinder.boundHalfDegrees(Robot.ahrs.getYaw() - 
        tapeAngle);
      previousYaw = previousRawYaw;
    }
    previousDistance = (Robot.driveSubsystem.getDistanceLeft() + Robot.driveSubsystem.getDistanceRight()) / 2;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double currentRawDistance = (Robot.driveSubsystem.getDistanceLeft() + Robot.driveSubsystem.getDistanceRight()) / 2;
    // Tape angle does not matter if vision angle available
    float currentRawYaw = (float)Pathfinder.boundHalfDegrees(Robot.ahrs.getYaw() - 
      (pipelineProcessor.providesVisionAngle() ? 0 : tapeAngle));
    if (pipelineProcessor.providesVisionAngle()) {
      angleData.addDataPoint(Pathfinder.boundHalfDegrees(angleData.getCurrentPoint() + (currentRawYaw - previousRawYaw)));
    } else {
      // If no vision angle data, do not use the method above that results in a starting point of 0
      angleData.addDataPoint(currentRawYaw);
    }
    // Apply distance moved in average angle
    updateXYData(currentRawDistance - previousDistance, 
      (angleData.getCurrentPoint() + previousYaw) / 2);
    previousDistance = currentRawDistance;
    previousRawYaw = currentRawYaw;
    previousYaw = angleData.getCurrentPoint();
    if (!pipeline.isDataHandled() && (!visionDataRecieved || distanceCalc.pidGet() > minCameraDistance)) {
      // System.out.println("Current time: " + Timer.getFPGATimestamp() + " Data time: " + pipeline.getLastTimestamp());
      // Calculate x and y using the pipeline processor
      boolean dataApplied = false;
      if (pipelineProcessor.process(pipeline)) {
        dataApplied = xData.addCorrectedData(pipelineProcessor.getX(), 
          pipeline.getLastTimestamp());
        dataApplied = dataApplied && yData.addCorrectedData(pipelineProcessor.getY(), 
          pipeline.getLastTimestamp());
        if (pipelineProcessor.providesVisionAngle()) {
          dataApplied = dataApplied && angleData.addCorrectedData(pipelineProcessor.getAngle(),
            pipeline.getLastTimestamp());
        }
        pipeline.dataHandled();
        // Start actually driving if this is the first data and
        // vision data has been incorporated into the LatencyData objs
        if (!visionDataRecieved && dataApplied) {
          turnController.enable();
          distanceController.enable();
          visionDataRecieved = true;
        }
      }
    }
    // Calculate a new angle setpoint
    turnController.setSetpoint(calcAngleSetpoint());
    // System.out.println("X: " + xData.getCurrentPoint());
    // System.out.println("Y: " + yData.getCurrentPoint());
    // System.out.println("Cur angle: " + angleData.getCurrentPoint());
    // System.out.println("Target angle: " + turnController.getSetpoint());
    // System.out.println("Distance: " + distanceCalc.pidGet());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return visionDataRecieved && Math.abs(distanceCalc.pidGet() - targetDistance) 
      <= distanceTolerance && distanceController.onTarget() && 
      turnController.onTarget() && Math.abs(angleData.getCurrentPoint()) <= 
      angleTolerance;
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
    double y = yData.getCurrentPoint();
    double targetY;
    if (useLookAheadNav) {
      targetY = y-lookAheadDistance;
      if (targetY > maxTargetY) {
        targetY = maxTargetY;
      }
    } else {
      if (distance >= maxTargetYDistance) {
        // map will scale out of range values so this enforces the maximum
        targetY = maxTargetY;
      } else {
        targetY = Robot.map(distance, 0, maxTargetYDistance, 0, maxTargetY);
      }
    }
    if (targetY < minTargetY) {
      targetY = minTargetY;
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

  private static abstract class PipelineProcessor {

    protected double x, y;
    protected Double angle = null;

    /**
     * Process the data from the pipeline
     * Note: Store x, y, angle in their local vars
     * @return Whether the processing was successful
     */
    public abstract boolean process(Pipeline pipeline);

    /**
     * Get the x value from the last call to process()
     * @return The last x value
     */
    public double getX() {return x;};

    /**
     * Get the y value from the last call to process()
     * @return The last y value
     */
    public double getY() {return y;};

    /**
     * Get the angle value from the last call to process().
     * Note: Pipeline processors that do not provide vision angle data may return null
     * @return The last angle (in world coords)
     */
    public Double getAngle() {return angle;};

    /**
     * Get whether the pipeline processor can provide vision-based angle information
     * @return Whether the processor can be used for vision-based angles
     */
    public abstract boolean providesVisionAngle();
  }

  @SuppressWarnings("unused")
  private class WhiteTapeProcessor extends PipelineProcessor {

    @Override
    public boolean process(Pipeline pipeline) {
      WhiteTapePipeline tapePipeline = (WhiteTapePipeline)pipeline;
      // Uses the gyro angle at the time the frame was taken in these calculations
      Double angleAtCapture = angleData.getPoint(tapePipeline.getLastTimestamp());
      if (angleAtCapture != null) {
        double angleToTarget = angleAtCapture
            + tapePipeline.getAngle();
        double distance = tapePipeline.getDistance();
        x = Math.sin(Math.toRadians(angleToTarget)) * distance;
        y = Math.cos(Math.toRadians(angleToTarget)) * distance;
        return true;
      }
      return false;
    }

    @Override
    public boolean providesVisionAngle() {
      return false;
    }
  }

  @SuppressWarnings("unused")
  private static class RetroSolvePnPProcessor extends PipelineProcessor {

    @Override
    public boolean process(Pipeline pipeline) {
      DeliveryTargetPipeline retroPipeline = (DeliveryTargetPipeline)pipeline;
      x = retroPipeline.getX();
      y = retroPipeline.getY();
      angle = retroPipeline.getAngle();
      return true;
    }

    @Override
    public boolean providesVisionAngle() {
      return true;
    }

  }
}
