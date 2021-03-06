package frc.robot.commands;

import frc.robot.util.PIDControllerFixed;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.DriveTrain.DriveGear;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Drive the specified distance on the specified heading
 * 
 * Does not turn first, really use it to drive straight
 * 
 * Also is not good if the robot is strongly knocked off course (like being
 * kicked)
 */
public class DriveDistanceOnHeading extends Command {

  private double kPDistance;
  private double kIDistance;
  private double kDDistance;
  private double kFDistance;
  private double kToleranceInches;
  static final int kToleranceBufSamplesDistance = 10;
  static final double kUpdatePeriodDistance = 0.02;

  private double kPAngle;
  private double kIAngle;
  private double kDAngle;
  private double kFAngle;
  private double kToleranceDegrees;
  static final int kToleranceBufSamplesAngle = 10;
  private double kTurnCorrectionAmount = 0.2;

  // PID output will be limited to negative to positive this. Multiplied by
  // RobotMap maxVelocity to get target
  private double kMaxOutput = 0.9;
  // Limit change in one iteration to this - % of max output
  private double kMaxChange = 0.03;
  // If robot does not move for this many cycles, give up
  private static final int stuckCycleThreshold = 50;
  // The maximum distance the robot can move in the number of cycles above to be
  // considered stuck
  private static final double stickDistance = 0.5;

  private DriveGear gear;

  private double maxOutputVelocityChange = kMaxOutput * kMaxChange;
  private PIDControllerFixed distanceController;
  private PIDControllerFixed turnController;
  private double targetDistance;
  private double targetAngle;
  private boolean useStartingYaw;
  private DistancePIDSource pidSourceDistance = new DistancePIDSource();
  private PIDOutputter pidOutputDistance = new PIDOutputter();
  private PIDOutputter pidOutputAngle = new PIDOutputter();
  private boolean resetCompletedDistance;
  private boolean resetStartedDistance;
  private double lastOutputDistance;
  private double lastDistance;
  private int stuckCycles;

  /**
   * Construct a new DriveDistanceOnHeading command using the starting heading
   * 
   * @param distance Distance to drive
   */
  public DriveDistanceOnHeading(double distance) {
    this(distance, 0);
    useStartingYaw = true;
  }

  /**
   * Construct a new DriveDistanceOnHeading command using the starting heading
   * 
   * @param distance        Distance to drive
   * @param toleranceInches How many inches off the end distance can be (pass 0 to
   *                        use default)
   * @param maxChange       Maximum percent velocity change per cycle (pass 0 to
   *                        use default)
   * @param maxOutput       Maximum percent velocity (pass 0 to use default)
   */
  public DriveDistanceOnHeading(double distance, double toleranceInches, double maxChange, double maxOutput) {
    this(distance, 0, toleranceInches, maxChange, maxOutput);
    useStartingYaw = true;
  }

  /**
   * Construct a new DriveDistanceOnHeading command with a defined heading
   * 
   * @param distance        Distance to drive
   * @param heading         Heading to try to follow
   * @param toleranceInches How many inches off the end distance can be (pass 0 to
   *                        use default)
   * @param maxChange       Maximum percent velocity change per cycle (pass 0 to
   *                        use default)
   * @param maxOutput       Maximum percent velocity (pass 0 to use default)
   */
  public DriveDistanceOnHeading(double distance, double heading, double toleranceInches, double maxChange,
      double maxOutput) {
    this(distance, heading);
    boolean maxPercentsChanged = false;
    if (maxOutput != 0) {
      kMaxOutput = maxOutput;
      kTurnCorrectionAmount *= kMaxOutput; // If a different max output is specified, calculate turn range as
      // percent of that
      maxPercentsChanged = true;
    }
    if (maxChange != 0) {
      kMaxChange = maxChange;
      maxPercentsChanged = true;
    }
    if (maxPercentsChanged) {
      maxOutputVelocityChange = kMaxOutput * kMaxChange;
    }
    if (toleranceInches != 0) {
      kToleranceInches = toleranceInches;
    }
  }

  /**
   * Construct a new DriveDistanceOnHeading command with a defined heading
   * 
   * @param distance Distance to drive
   * @param heading  Heading to try to follow
   */
  public DriveDistanceOnHeading(double distance, double heading) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    super("DriveDistanceOnHeading");
    requires(Robot.driveSubsystem);
    targetDistance = distance;
    targetAngle = (heading > 180) ? 180 : heading;
    targetAngle = (targetAngle < -180) ? -180 : targetAngle;
    useStartingYaw = false;
    switch (RobotMap.robot) {
    case ROBOT_2017:
      kPDistance = 0.032;
      kIDistance = 0.000000;
      kDDistance = 0;
      kFDistance = 0;
      kToleranceInches = 0.5;
      kPAngle = 0.05; // was 0.05, disabled due to WPILib Tolerance buffer phase lag
      kIAngle = 0;
      kDAngle = 0;
      kFAngle = 0;
      kToleranceDegrees = 1;
      break;
    case ORIGINAL_ROBOT_2018:
      kPDistance = 0.02;
      kIDistance = 0.000000;
      kDDistance = 0;
      kFDistance = 0;
      kToleranceInches = 0.5;
      kPAngle = 0.05; // was 0.05, disabled due to WPILib Tolerance buffer phase lag
      kIAngle = 0;
      kDAngle = 0;
      kFAngle = 0;
      kToleranceDegrees = 1;
      gear = DriveGear.HIGH;
      break;
    case EVERYBOT_2019:
      kPDistance = 0.017;
      kIDistance = 0;
      kDDistance = 0;
      kFDistance = 0.5;
      kToleranceInches = 0.5;
      kPAngle = 0.07;
      kIAngle = 0;
      kDAngle = 0;
      kFAngle = 0;
      kToleranceDegrees = 0.5;
      break;
    case ROBOT_2019:
    case ROBOT_2019_2:
      kPDistance = 0.014;
      kIDistance = 0.000000;
      kDDistance = 0;
      kFDistance = 0;
      kToleranceInches = 0.5;
      kPAngle = 0.05;
      kIAngle = 0;
      kDAngle = 0;
      kFAngle = 0;
      kToleranceDegrees = 1;
      break;
    default:
      break;
    }
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    if (Robot.driveSubsystem.isDualGear()) {
      Robot.driveSubsystem.switchGear(gear);
    }
    if (useStartingYaw) {
      targetAngle = Robot.ahrs.getYaw();
    }

    // For reseting encoders, wheels need to not be moving, but they could be
    // commanded to move from a previous command
    Robot.driveSubsystem.drive(0, 0);

    distanceController = new PIDControllerFixed(kPDistance, kIDistance, kDDistance, kFDistance, pidSourceDistance,
        pidOutputDistance, kUpdatePeriodDistance);
    turnController = new PIDControllerFixed(kPAngle, kIAngle, kDAngle, kFAngle, Robot.ahrs, pidOutputAngle);
    distanceController.setOutputRange(-1, 1);
    turnController.setOutputRange(-1, 1);
    distanceController.setAbsoluteTolerance(kToleranceInches);
    distanceController.setToleranceBuffer(kToleranceBufSamplesDistance);
    distanceController.setSetpoint(targetDistance);
    turnController.setInputRange(-180.0f, 180.0f);
    turnController.setOutputRange(-1.0, 1.0);
    turnController.setAbsoluteTolerance(kToleranceDegrees);
    turnController.setToleranceBuffer(kToleranceBufSamplesAngle);
    turnController.setContinuous(true);
    turnController.setSetpoint(targetAngle);
    lastOutputDistance = 0;
    resetCompletedDistance = false;
    resetStartedDistance = false;
    stuckCycles = 0;
  }

  private double calcNewVelocity(PIDOutputter pidOut, double lastOutput) {
    double targetOutput = pidOut.getPIDRate() * (kMaxOutput - kTurnCorrectionAmount);
    if (Math.abs(lastOutput - targetOutput) > maxOutputVelocityChange) {
      if (lastOutput < targetOutput) {
        targetOutput = lastOutput + maxOutputVelocityChange;
      } else {
        targetOutput = lastOutput - maxOutputVelocityChange;
      }
    }
    return targetOutput;
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
    if (!resetStartedDistance && Math.abs(Robot.driveSubsystem.getVelocityLeft()) < 1
        && Math.abs(Robot.driveSubsystem.getVelocityRight()) < 1) {
      Robot.driveSubsystem.resetPosition();
      resetStartedDistance = true;
    }
    if (resetStartedDistance && Math.abs(Robot.driveSubsystem.getDistanceLeft()) < 0.1
        && Math.abs(Robot.driveSubsystem.getDistanceRight()) < 0.1) {
      resetCompletedDistance = true;
      distanceController.enable();
      turnController.enable();
    }
    if (resetCompletedDistance) {
      double outputVelocity = calcNewVelocity(pidOutputDistance, lastOutputDistance);
      lastOutputDistance = outputVelocity;
      double outputTurnVelocity = pidOutputAngle.getPIDRate() * kTurnCorrectionAmount;
      // subtract from right side, add to left side (drive left on positive)
      Robot.driveSubsystem.drive(outputVelocity + outputTurnVelocity, outputVelocity - outputTurnVelocity);
      if (RobotMap.tuningMode) {
        SmartDashboard.putString("Distance Graph", Robot.genGraphStr(getAverageDistance(), targetDistance));
        SmartDashboard.putNumber("Current Distance", getAverageDistance());
        SmartDashboard.putNumber("Target Distance", targetDistance);
        SmartDashboard.putString("Velocity Graph", Robot.genGraphStr(outputVelocity,
            outputVelocity + outputTurnVelocity, outputVelocity - outputTurnVelocity));
        SmartDashboard.putNumber("Center Velocity", outputVelocity);
        SmartDashboard.putNumber("Left Velocity", outputVelocity + outputTurnVelocity);
        SmartDashboard.putNumber("Right Velocity", outputVelocity - outputTurnVelocity);
        SmartDashboard.putNumber("Turn controller output", pidOutputAngle.getPIDRate());
        SmartDashboard.putString("Yaw Graph", Robot.genGraphStr(targetAngle, Robot.ahrs.getYaw()));
        SmartDashboard.putNumber("Target yaw", targetAngle);
        SmartDashboard.putNumber("Current Angle", Robot.ahrs.getYaw());
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    if (Math.abs(getAverageDistance() - lastDistance) <= stickDistance) {
      stuckCycles++;
    } else {
      stuckCycles = 0;
      lastDistance = getAverageDistance();
    }
    return resetCompletedDistance && ((distanceController.onTarget() && turnController.onTarget()
        && (Math.abs(getAverageDistance() - targetDistance) < kToleranceInches)
        && (Math.abs(Robot.ahrs.getYaw() - targetAngle) < kToleranceDegrees)) || stuckCycles >= stuckCycleThreshold);
  }

  // Called once after isFinished returns true
  protected void end() {
    turnController.disable();
    distanceController.disable();
    Robot.driveSubsystem.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
  }

  private double getAverageDistance() {
    return (Robot.driveSubsystem.getDistanceLeft() + Robot.driveSubsystem.getDistanceRight()) / 2;
  }

  private class DistancePIDSource implements PIDSource {
    @Override
    public double pidGet() {
      return getAverageDistance();
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {

    }

    @Override
    public PIDSourceType getPIDSourceType() {
      return PIDSourceType.kDisplacement;
    }
  }

  private class PIDOutputter implements PIDOutput {
    private double PIDRate;

    public double getPIDRate() {
      return PIDRate;
    }

    @Override
    public void pidWrite(double output) {
      PIDRate = output;
    }
  }
}
