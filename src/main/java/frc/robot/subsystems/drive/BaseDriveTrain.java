package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI.OIType;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.subsystems.DriveTrain.DriveGear;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public abstract class BaseDriveTrain<currentControlMode> extends Subsystem {

  private double kPLow;
  private double kILow;
  private double kDLow;
  private double kFLow;
  private int kIZoneLow;
  private double kPHigh;
  private double kIHigh;
  private double kDHigh;
  private double kFHigh;
  private int kIZoneHigh;
  private double kPMP; // the MP settings are also used for distance close loop
  private double kIMP;
  private double kDMP;
  private double kFMP;
  private int kIZoneMP;
  private int kAllowableErrorDistance; // ticks sent to talon as allowable error for distance close loop
  private double kPPTO;
  private double kIPTO;
  private double kDPTO;
  private int kIZonePTO;
  private boolean PTOUseMotMaj;

  private static final boolean dualGear = false;
  private static final DriveGear currentGear = null;
  @SuppressWarnings("unused")
  private static final int requiredTrajPoints = 5; // point to send to talon before starting profile

  private DoubleSolenoid leftGearSolenoid;
  private DoubleSolenoid rightGearSolenoid;
  private int ticksPerRotation; // getEncPosition values in one turn
  private double wheelDiameter; // inches
  @SuppressWarnings("unused")
  private double wheelBaseWidth; // inches, distance between left and right wheels
  private double nominalOutputVoltage;
  private DriveControlMode currentControlMode = DriveControlMode.STANDARD_DRIVE; // enum defined at end of file
  private boolean sixMotorDrive = false;
  private boolean hasPTO = false;
  private boolean followersAreVictorSPX = true;
  private DoubleSolenoid pto;
  private Double PTOLeftStartingPosition;
  private Double PTORightStartingPosition;
  private double PTORightSpeedAdjust; // Multiplier applied to right side setpoint when driving the PTO
  private double PTOLeftSpeedAdjust;
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveWithJoystick());
  }

  /**
   * Drive the robot with speed specified as inches per second
   * 
   * @param left  Left inches per second
   * @param right Right inches per second
   */
  public void driveInchesPerSec(final int left, final int right) {
    driveInchesPerSec((double) right, (double) left);
  }

  /**
   * Drive the robot with speed specified as inches per second
   * 
   * @param left  Left inches per second
   * @param right Right inches per second
   */
  public void driveInchesPerSec(final double left, final double right) {
    int maxVelocity;
    if (!dualGear || currentGear == DriveGear.LOW) {
      maxVelocity = RobotMap.maxVelocityLow;
    } else {
      maxVelocity = RobotMap.maxVelocityHigh;
    }
    drive((left / (wheelDiameter * Math.PI)) * ticksPerRotation / 10 / maxVelocity,
        (right / (wheelDiameter * Math.PI)) * ticksPerRotation / 10 / maxVelocity);
  }

  private double calcActualVelocity(final double input) {
    int minVelocity;
    if (!dualGear || currentGear == DriveGear.LOW) {
      minVelocity = RobotMap.minVelocityLow;
    } else {
      minVelocity = RobotMap.minVelocityHigh;
    }
    if (input >= -0.1 && input <= 0.1) {
      return 0;
    } else if (input > 0.1 && input < minVelocity) {
      return minVelocity;
    } else if (input < -0.1 && input > minVelocity * -1) {
      return minVelocity * -1;
    } else {
      return input;
    }
  }

  /**
   * Make the robot drive
   * 
   * @param left  Left percent speed
   * @param right Right percent speed
   */
  public void drive(final double left, final double right) {

  }

  /**
   * Make the robot drive
   * 
   * @param left             Left percent speed
   * @param right            Right percent speed
   * @param alwaysHighMaxVel Whether to always use the max velocity of high gear
   *                         or of current gear
   */
  public void drive(double left, double right, final boolean alwaysHighMaxVel) {
    if (!Robot.oi.getDriveEnabled()) {
      left = 0;
      right = 0;
    }
    if (currentControlMode == DriveControlMode.STANDARD_DRIVE) {
      int maxVelocity;
      // actualMaxVelocity is used to make open loop scaling accurate
      int actualMaxVelocity;
      if (!dualGear) {
        maxVelocity = RobotMap.maxVelocityLow;
        actualMaxVelocity = RobotMap.maxVelocityLow;
      } else if (currentGear == DriveGear.LOW) {
        if (!alwaysHighMaxVel) {
          maxVelocity = RobotMap.maxVelocityLow;
        } else {
          maxVelocity = RobotMap.maxVelocityHigh;
        }
        actualMaxVelocity = RobotMap.maxVelocityLow;
      } else {
        maxVelocity = RobotMap.maxVelocityHigh;
        actualMaxVelocity = RobotMap.maxVelocityHigh;
      }
      left *= maxVelocity;
      right *= maxVelocity;
      left = calcActualVelocity(left);
      right = calcActualVelocity(right);

      if (Robot.oi.getOpenLoop()) {
      }
    }

  }

  public void stop() {
    drive(0, 0);
  }

  protected abstract void neutralOutput();

  protected abstract void openLoop();

  protected abstract void closedLoop();

  public void enableBrakeMode(final boolean enable) {

  }

  public abstract void resetPosition() {

  }

  public double getRotationsLeft() {

  }

  public double getRotationsRight() {

  }

  public double getDistanceRight() {
    return wheelDiameter * Math.PI * getRotationsRight();
  }

  public double getDistanceLeft() {
    return wheelDiameter * Math.PI * getRotationsLeft();
  }

  /**
   * Get the current velocity for the right side of the robot
   * 
   * @return current velocity in inches per second
   */
  public abstract double getVelocityRight() {

  }

  /**
   * Get the current velocity for the left side of the robot
   * 
   * @return current velocity in inches per second
   */
  public abstract double getVelocityLeft() {

  }

  public abstract double getCurrent() {

  }

  /**
   * Sets the PID parameters for the current control mode, useful for tuning.
   * Calling this effects everything using the subsystem, use with care.
   * 
   * @param p     P
   * @param i     I
   * @param d     D
   * @param f     F
   * @param iZone Integral zone
   */
  public abstract void setPID(final double p, final double i, final double d, final double f, final int iZone) {

  }

  /**
   * Sets the PID parameters for the given slot on the talon, used during setup
   * 
   * @param slotIdx Which slot to write to
   * @param p       P
   * @param i       I
   * @param d       D
   * @param f       F
   * @param iZone   Integral zone
   */
  private abstract void setPID(final int slotIdx, final double p, final double i, final double d, final double f, final int iZone) {

  }

  public abstract double getP() {

  }

  public abstract double getI() {

  }

  public abstract double getD() {

  }

  public abstract void getF() {

  }

  public abstract void changeStatusRate(final int ms) {

  }

  public abstract void resetSensorRate() {

  }

  public abstract void changeControlRate(final int ms) {

  }

  public abstract void resetControlRate() {

  }

  public void switchGear(final DriveGear gear) {
    if (dualGear && (Robot.oi == null || Robot.oi.isShiftingEnabled())) {
      switch (gear) {
      case HIGH:
        leftGearSolenoid.set(Value.kForward);
        rightGearSolenoid.set(Value.kForward);
        currentGear = DriveGear.HIGH;
        SmartDashboard.putBoolean("High Gear", true);
        break;
      case LOW:
        leftGearSolenoid.set(Value.kReverse);
        rightGearSolenoid.set(Value.kReverse);
        currentGear = DriveGear.LOW;
        SmartDashboard.putBoolean("High Gear", false);
        break;
      case UNSUPPORTED:
      default:
        break;
      }
    }
  }

  public DriveGear getCurrentGear() {
    if (dualGear) {
      return currentGear;
    } else {
      return DriveGear.UNSUPPORTED;
    }
  }

  public boolean isDualGear() {
    return dualGear;
  }

  public void enablePTO() {
    currentControlMode = DriveControlMode.PTO;
    neutralOutput();
    pto.set(Value.kForward);
  }

  public void disablePTO(Object DriveControlMode) {
    if (currentControlMode == DriveControlMode.PTO) {
      neutralOutput();
      pto.set(Value.kReverse);
      currentControlMode = DriveControlMode.STANDARD_DRIVE;
    }
  }

  public void runPTO(final double speed) {
    if (Robot.oi.getDriveEnabled() && currentControlMode == DriveControlMode.PTO) {
      openLoop(speed * PTOLeftSpeedAdjust, speed * PTORightSpeedAdjust);
    } else if (!Robot.oi.getDriveEnabled()) {
      neutralOutput();
    }
  }
}