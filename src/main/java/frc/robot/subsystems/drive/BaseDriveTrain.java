package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithJoystick;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public abstract class BaseDriveTrain extends Subsystem {

  protected double kPLow;
  protected double kILow;
  protected double kDLow;
  protected double kFLow;
  protected int kIZoneLow;
  protected double kPHigh;
  protected double kIHigh;
  protected double kDHigh;
  protected double kFHigh;
  protected int kIZoneHigh;

  protected double wheelDiameter; // inches
  protected boolean dualGear = false;
  protected boolean hasPTO = false;
  protected double PTORightSpeedAdjust; // Multiplier applied to right side setpoint when driving the PTO
  protected double PTOLeftSpeedAdjust;

  private DoubleSolenoid pto;
  private DoubleSolenoid leftGearSolenoid;
  private DoubleSolenoid rightGearSolenoid;

  private DriveControlMode currentControlMode = DriveControlMode.STANDARD_DRIVE; // enum defined at end of file
  private DriveGear currentGear = null;

  public BaseDriveTrain() {
    if (dualGear) {
      leftGearSolenoid = new DoubleSolenoid(RobotMap.leftDriveGearPCM, RobotMap.leftDriveGearSolenoid1,
          RobotMap.leftDriveGearSolenoid2);
      rightGearSolenoid = new DoubleSolenoid(RobotMap.rightDriveGearPCM, RobotMap.rightDriveGearSolenoid1,
          RobotMap.rightDriveGearSolenoid2);
      setPID(1, kPHigh, kIHigh, kDHigh, kFHigh, kIZoneHigh);
      switchGear(DriveGear.HIGH);
    }
    setPID(0, kPLow, kILow, kDLow, kFLow, kIZoneLow);
    if (hasPTO) {
      pto = new DoubleSolenoid(RobotMap.ptoSolenoidPCM, RobotMap.ptoSolenoid1, RobotMap.ptoSolenoid2);
      disablePTO();
    }
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveWithJoystick());
  }

  public double getMaxVelocity() {
    if (!dualGear || currentGear == DriveGear.LOW) {
      return RobotMap.maxVelocityLow;
    } else {
      return RobotMap.maxVelocityHigh;
    }
  }

  /**
   * Drive the robot with speed specified as inches per second
   * 
   * @param left  Left inches per second
   * @param right Right inches per second
   */
  public void driveInchesPerSec(int left, int right) {
    driveInchesPerSec((double) right, (double) left);
  }

  /**
   * Drive the robot with speed specified as inches per second
   * 
   * @param left  Left inches per second
   * @param right Right inches per second
   */
  public void driveInchesPerSec(double left, double right) {
    if (currentControlMode == DriveControlMode.STANDARD_DRIVE) {
      if (!Robot.oi.getDriveEnabled()) {
        left = 0;
        right = 0;
      }

      if (Robot.oi.getOpenLoop()) {
        driveOpenLoopLowLevel(calcActualVelocity(left, false) / getMaxVelocity(),
            calcActualVelocity(right, false) / getMaxVelocity());
      } else {
        driveClosedLoopLowLevel((calcActualVelocity(left, false) / (wheelDiameter * Math.PI)),
            (calcActualVelocity(right, false) / (wheelDiameter * Math.PI)));
      }
    }
  }

  /**
   * Increases inputs that are less than minVelocity to minVelocity.
   * 
   * @param input        Input value, either inches per second or percentage
   * @param isPercentage Whether the input is a percentage
   * @return Calculated velocity
   */
  private double calcActualVelocity(double input, boolean isPercentage) {
    double minVelocity;
    if (!dualGear || currentGear == DriveGear.LOW) {
      minVelocity = RobotMap.minVelocityLow;
    } else {
      minVelocity = RobotMap.minVelocityHigh;
    }
    double minNonZero = 0.1;
    if (isPercentage) {
      minVelocity /= getMaxVelocity();
      minNonZero = 0.01;
    }
    if (input > minNonZero * -1 && input < minNonZero) {
      return 0;
    } else if (input >= minNonZero && input < minVelocity) {
      return minVelocity;
    } else if (input <= minNonZero * -1 && input > minVelocity * -1) {
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
  public void drive(double left, double right) {
    drive(left, right, false);
  }

  /**
   * Make the robot drive
   * 
   * @param left             Left percent speed
   * @param right            Right percent speed
   * @param alwaysHighMaxVel Whether to always use the max velocity of high gear
   *                         or of current gear
   */
  public void drive(double left, double right, boolean alwaysHighMaxVel) {
    if (currentControlMode == DriveControlMode.STANDARD_DRIVE) {
      if (!Robot.oi.getDriveEnabled()) {
        left = 0;
        right = 0;
      }

      double maxVelocity = getMaxVelocity();
      if (dualGear && alwaysHighMaxVel && currentGear == DriveGear.LOW) {
        maxVelocity = RobotMap.maxVelocityHigh;
      }
      left = calcActualVelocity(left, true);
      right = calcActualVelocity(right, true);
      // If in closed loop, convert max velocity from inches per second to rotations
      // per second to match input unit of driveClosedLoopLowLevel
      if (!Robot.oi.getOpenLoop()) {
        maxVelocity /= wheelDiameter * Math.PI;
      }
      left *= maxVelocity;
      right *= maxVelocity;

      if (Robot.oi.getOpenLoop()) {
        driveOpenLoopLowLevel(left / getMaxVelocity(), right / getMaxVelocity());
      } else {
        driveClosedLoopLowLevel(left, right);
      }
    }
  }

  public void stop() {
    drive(0, 0);
  }

  protected abstract void neutralOutput();

  protected abstract void driveOpenLoopLowLevel(double left, double right);

  protected abstract void driveClosedLoopLowLevel(double left, double right);

  public abstract void enableBrakeMode(boolean enable);

  public abstract void resetPosition();

  public abstract double getRotationsLeft();

  public abstract double getRotationsRight();

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
  public abstract double getVelocityRight();

  /**
   * Get the current velocity for the left side of the robot
   * 
   * @return current velocity in inches per second
   */
  public abstract double getVelocityLeft();

  public abstract double getCurrent();

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
  public abstract void setPID(double p, double i, double d, double f, int iZone);

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
  protected abstract void setPID(int slotIdx, double p, double i, double d, double f, int iZone);

  public abstract double getP();

  public abstract double getI();

  public abstract double getD();

  public abstract void getF();

  public abstract void changeStatusRate(int ms);

  public abstract void resetSensorRate();

  public abstract void changeControlRate(int ms);

  public abstract void resetControlRate();

  protected abstract void setProfileSlot(int slotIdx);

  public void switchGear(DriveGear gear) {
    if (dualGear && (Robot.oi == null || Robot.oi.isShiftingEnabled())) {
      switch (gear) {
      case HIGH:
        leftGearSolenoid.set(Value.kForward);
        rightGearSolenoid.set(Value.kForward);
        setProfileSlot(1);
        currentGear = DriveGear.HIGH;
        SmartDashboard.putBoolean("High Gear", true);
        break;
      case LOW:
        leftGearSolenoid.set(Value.kReverse);
        rightGearSolenoid.set(Value.kReverse);
        setProfileSlot(0);
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
    if (hasPTO) {
      currentControlMode = DriveControlMode.PTO;
      neutralOutput();
      pto.set(Value.kForward);
    }
  }

  public void disablePTO() {
    if (currentControlMode == DriveControlMode.PTO) {
      neutralOutput();
      pto.set(Value.kReverse);
      currentControlMode = DriveControlMode.STANDARD_DRIVE;
    }
  }

  public void runPTO(double speed) {
    if (Robot.oi.getDriveEnabled() && currentControlMode == DriveControlMode.PTO) {
      driveOpenLoopLowLevel(speed * PTOLeftSpeedAdjust, speed * PTORightSpeedAdjust);
    } else if (!Robot.oi.getDriveEnabled()) {
      neutralOutput();
    }
  }

  private enum DriveControlMode {
    STANDARD_DRIVE, PTO
  }

  public enum DriveGear {
    HIGH, LOW, UNSUPPORTED
  }
}