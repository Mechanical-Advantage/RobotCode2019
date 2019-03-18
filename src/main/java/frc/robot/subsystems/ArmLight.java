/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.RobotType;
import frc.robot.util.SchoolZone;
import frc.robot.util.TunableNumber;

/**
 * 2019 robot light arm
 */
public class ArmLight extends Subsystem {

  private static final FeedbackDevice elbowSensorType = FeedbackDevice.CTRE_MagEncoder_Relative;
  private static final int elbowTicksPerRotation = 4096;
  private static final boolean elbowSensorLeftReversed = true;
  private static final boolean elbowOutputLeftReversed = false;
  private static final boolean elbowUseMotionMagic = true;
  private static final double elbowMotMagAccel = 20;
  private static final double elbowMotMagCruiseVelocity = 70;
  private static final double elbowZeroedPosition = 0; // deg
  private static final double elbowLowerLimitLow = 0;
  private static final double elbowUpperLimitLow = 360;
  private static final double elbowLowerLimitHigh = 0;
  private static final double elbowUpperLimitHigh = 360;
  private static final double elbowReduction = 1.5 * 33 * 2.72; // Multiplier on setpoints
  private static final double elbowOffsetLow = 0; // Elbow offset applied when shoulder is lowered
  private static final double elbowOffsetHigh = 60; // Elbow offset applied when shoulder is raised
  private static final double elbowSchoolZoneSpeedLimit = 0.2;
  private static final double elbowLowSchoolZoneLowerStart = -360;
  private static final double elbowLowSchoolZoneUpperStart = 360;
  private static final double elbowHighSchoolZoneLowerStart = -360;
  private static final double elbowHighSchoolZoneUpperStart = 360;
  private static final double elbowPeakOutput = 1;
  private static final double elbowForwardNominalOutput = 0.12;
  private static final double elbowReverseNominalOutput = -0.12;
  private static final double elbowAllowableError = 0; // For primary PID
  private static final double elbowZeroPercent = -0.05;
  private static final double elbowStartingPosition = 0;
  private static final NeutralMode elbowNeutralMode = NeutralMode.Brake;

  private static final boolean driveToZeroStartup = false;

  private static final TunableNumber kPElbow = new TunableNumber("Arm Elbow/p");
  private static final TunableNumber kIElbow = new TunableNumber("Arm Elbow/i");
  private static final TunableNumber kDElbow = new TunableNumber("Arm Elbow/d");

  private static final boolean elbowEnableCurrentLimit = true;
  private static final int elbowContinousCurrentLimit = 40;
  private static final int elbowPeakCurrentLimit = 50;
  private static final int elbowPeakCurrentLimitDuration = 3000; // ms

  private TalonSRX elbowLeft;
  private DoubleSolenoid shoulder1;
  private DoubleSolenoid shoulder2;
  private DigitalInput elbowLimitSwitch;

  private boolean shoulderRaised = false;
  private boolean targetShoulderRaised = false; // State requested for the shoulder
  private double targetElbowPosition;
  private boolean elbowZeroed = false;
  private SchoolZone elbowLowSchoolZone;
  private SchoolZone elbowHighSchoolZone;
  private SchoolZone elbowCurrentSchoolZone;
  private boolean elbowEnabled;
  private boolean elbowLimitsEnabled = true;
  private boolean elbowDownEnabledLast = true;

  public ArmLight() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      kPElbow.setDefault(1.3);
      kIElbow.setDefault(0);
      kDElbow.setDefault(13);

      shoulder1 = new DoubleSolenoid(RobotMap.armShoulder1PCM, RobotMap.armShoulder1Extend,
          RobotMap.armShoulder1Retract);
      shoulder2 = new DoubleSolenoid(RobotMap.armShoulder2PCM, RobotMap.armShoulder2Extend,
          RobotMap.armShoulder2Retract);

      elbowLimitSwitch = new DigitalInput(RobotMap.armElbowLimitSwitch);

      elbowLeft = new TalonSRX(RobotMap.armElbowLeft);

      elbowLowSchoolZone = new SchoolZone(elbowSchoolZoneSpeedLimit, elbowPeakOutput, elbowLowSchoolZoneLowerStart,
          elbowLowSchoolZoneUpperStart, elbowLeft);
      elbowHighSchoolZone = new SchoolZone(elbowSchoolZoneSpeedLimit, elbowPeakOutput, elbowHighSchoolZoneLowerStart,
          elbowHighSchoolZoneUpperStart, elbowLeft);

      elbowLeft.configFactoryDefault();
      elbowLeft.configSelectedFeedbackSensor(elbowSensorType);
      elbowLeft.setSensorPhase(elbowSensorLeftReversed);
      elbowLeft.setInverted(elbowOutputLeftReversed);
      elbowLeft.setNeutralMode(elbowNeutralMode);

      initPID();

      elbowLeft.selectProfileSlot(0, 0); // Use slot 0 for main PID
      elbowLeft.configForwardSoftLimitEnable(true);
      elbowLeft.configReverseSoftLimitEnable(true);
      elbowLeft.configMotionCruiseVelocity(convertElbowPositionToTicks(elbowMotMagCruiseVelocity, false));
      elbowLeft.configMotionAcceleration(convertElbowPositionToTicks(elbowMotMagAccel, false));
      elbowLeft.configNominalOutputForward(elbowForwardNominalOutput);
      elbowLeft.configNominalOutputReverse(elbowReverseNominalOutput);
      elbowLeft.configAllowableClosedloopError(0, convertElbowPositionToTicks(
        elbowAllowableError, false));
      elbowLowSchoolZone.setControllerLimits(); // This sets the peak output of the controllers

      elbowLeft.configContinuousCurrentLimit(elbowContinousCurrentLimit);
      elbowLeft.configPeakCurrentLimit(elbowPeakCurrentLimit);
      elbowLeft.configPeakCurrentDuration(elbowPeakCurrentLimitDuration);
      elbowLeft.enableCurrentLimit(elbowEnableCurrentLimit);

      // enableElbow();
      disableElbow();
      setShoulderRaised(false); // Will set elbow limits as well and ensures consistent state

      // Start zeroing the mechanisms
      // Normal control will not work until this is complete (see periodic())
      if (driveToZeroStartup) {
        zeroElbow();
      }
    }
  }

  @Override
  public void periodic() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      if (RobotMap.tuningMode) {
        initPID();
      }
      // Should zeroing multiple times be allowed?
      double elbowPosition = getElbowPosition(); // Avoid duplicate calculation
      updateShoulderSetpoint(elbowPosition);
      if (getElbowLimitSwitch()) {
        setElbowZeroed();
        if (elbowDownEnabledLast) {
          elbowDownEnabledLast = false;
          elbowLeft.configPeakOutputReverse(0);
        }
      } else if (!elbowDownEnabledLast) {
        elbowDownEnabledLast = true;
        // Reset peak outputs to normal values
        elbowCurrentSchoolZone.setControllerLimits();
      }
      if (elbowZeroed) {
        // Disable applying of reverse limits if elbow down disabled because that also uses peak output
        elbowCurrentSchoolZone.applyPosition(elbowPosition, true, elbowDownEnabledLast);
      }
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  private void initPID() {
    elbowLeft.config_kP(0, kPElbow.get());
    elbowLeft.config_kI(0, kIElbow.get());
    elbowLeft.config_kD(0, kDElbow.get());
  }

  /**
   * Sets the encoder tick counts on the talons to the starting positions
   */
  public void setToStartingPosition() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      elbowLeft.setSelectedSensorPosition(convertElbowPositionToTicks(elbowStartingPosition, false));
      elbowZeroed = true;
    }
  }

  public void setShoulderRaised(boolean raise) {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      targetShoulderRaised = raise;
      updateShoulderSetpoint();
      // May need to update setpoints for other parts here
      updateElbowLimits();
      updateElbowSetpoint();
    }
  }

  /**
   * Get whether the shoulder is raised
   * 
   * @return Whether the shoulder is currently raised
   */
  public boolean isShoulderRaised() {
    return shoulderRaised;
  }

  /**
   * Get whether the requested shoulder position is raised
   * 
   * @return Whether the target shoulder position is raised
   */
  public boolean isShoulderTargetRaised() {
    return targetShoulderRaised;
  }

  private void updateShoulderSetpoint() {
    updateShoulderSetpoint(getElbowPosition());
  }

  private void updateShoulderSetpoint(double elbowPosition) {
    if ((Robot.oi == null || Robot.oi.isArmEnabled()) && RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      if (targetShoulderRaised != shoulderRaised) {
        if (elbowPosition >= getElbowLowerLimit(targetShoulderRaised) && 
        elbowPosition <= getElbowUpperLimit(targetShoulderRaised)) {
          shoulder1.set(targetShoulderRaised ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
          shoulder2.set(targetShoulderRaised ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
          shoulderRaised = targetShoulderRaised;  
        }
      }
    }
  }

  /**
   * Set the elbow position
   * 
   * @param position How many degrees from forward parallel to the floor
   */
  public void setElbowPosition(double position) {
    targetElbowPosition = position;
    updateElbowSetpoint();
  }

  /**
   * Get the target position of the elbow
   * 
   * @return The target elbow position in degrees from floor
   */
  public double getElbowTargetPosition() {
    return targetElbowPosition;
  }

  /**
   * Get the actual position of the elbow
   * 
   * @return The elbow position in degrees from floor
   */
  public double getElbowPosition() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      // Average position
      double position = elbowLeft.getSelectedSensorPosition();
      position = position / elbowReduction / elbowTicksPerRotation * 360; // Convert to degrees
      if (shoulderRaised) {
        position = position - elbowOffsetHigh;
      } else {
        position = position - elbowOffsetLow;
      }
      return position;
    } else {
      return 0;
    }
  }

  private void updateElbowSetpoint() {
    if ((elbowZeroed || !elbowLimitsEnabled) && (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2)
        && elbowEnabled) {
      double target;
      if (elbowLimitsEnabled && targetElbowPosition < getElbowLowerLimit(targetShoulderRaised)) {
        target = getElbowLowerLimit(targetShoulderRaised);
      } else if (elbowLimitsEnabled && targetElbowPosition > getElbowUpperLimit(targetShoulderRaised)) {
        target = getElbowUpperLimit(targetShoulderRaised);
      } else {
        target = targetElbowPosition;
      }
      elbowLeft.set(elbowUseMotionMagic ? ControlMode.MotionMagic : ControlMode.Position,
          convertElbowPositionToTicks(target, true), DemandType.Neutral, 0);
    }
  }

  private int convertElbowPositionToTicks(double position, boolean enableOffsets) {
    double setpoint;
    if (enableOffsets) {
      if (shoulderRaised) {
        setpoint = position + elbowOffsetHigh;
      } else {
        setpoint = position + elbowOffsetLow;
      }
    } else {
      setpoint = position;
    }
    setpoint = setpoint / 360 * elbowTicksPerRotation; // Convert degrees to ticks
    setpoint *= elbowReduction;
    return (int) Math.round(setpoint);
  }

  private void updateElbowLimits() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      if (shoulderRaised) {
        elbowCurrentSchoolZone = elbowHighSchoolZone;
      } else {
        elbowCurrentSchoolZone = elbowLowSchoolZone;
      }
      // Ensure school zones are applied if needed
      elbowCurrentSchoolZone.applyPosition(getElbowPosition());
      elbowCurrentSchoolZone.setControllerLimits();
      int finalLowerLimit = (int) Math.round(getElbowLowerLimit() / 360 * elbowTicksPerRotation * elbowReduction);
      int finalUpperLimit = (int) Math.round(getElbowUpperLimit() / 360 * elbowTicksPerRotation * elbowReduction);
      elbowLeft.configForwardSoftLimitThreshold(finalUpperLimit);
      elbowLeft.configReverseSoftLimitThreshold(finalLowerLimit);
    }
  }

  /**
   * Get the lower elbow limit for the current shoulder state
   * 
   * @return The elbow limit (degrees from floor)
   */
  private double getElbowLowerLimit() {
    return getElbowLowerLimit(shoulderRaised);
  }
  /**
   * Get the lower elbow limit for the passed state
   * 
   * @param raised The shoulder state to get the limit for
   * @return The elbow limit (degrees from floor)
   */
  private double getElbowLowerLimit(boolean raised) {
    return raised ? elbowLowerLimitHigh : elbowLowerLimitLow;
  }
  /**
   * Get the upper elbow limit for the current shoulder state
   * 
   * @return The elbow limit (degrees from floor)
   */
  private double getElbowUpperLimit() {
    return getElbowUpperLimit(shoulderRaised);
  }
  /**
   * Get the upper elbow limit for the passed state
   * 
   * @param raised The shoulder state to get the limit for
   * @return The elbow limit (degrees from floor)
   */
  private double getElbowUpperLimit(boolean raised) {
    return raised ? elbowUpperLimitHigh : elbowUpperLimitLow;
  }

  /**
   * Set the elbow to be zeroed
   */
  private void setElbowZeroed() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      int newPosition = (int) Math.round(elbowZeroedPosition / 360 * elbowTicksPerRotation * elbowReduction);
      elbowLeft.setSelectedSensorPosition(newPosition);
      elbowZeroed = true;
      enableElbowLimits();
      elbowLeft.overrideSoftLimitsEnable(true);
      updateElbowSetpoint();
    }
  }

  public void zeroElbow() {
    if (elbowEnabled && (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2)) {
      elbowZeroed = false;
      disableElbowLimits();
      elbowLeft.set(ControlMode.PercentOutput, elbowZeroPercent, DemandType.Neutral, 0);
    }
  }

  private void disableElbowLimits() {
    elbowLimitsEnabled = false;
    elbowLeft.overrideSoftLimitsEnable(false);
  }
  private void enableElbowLimits() {
    elbowLimitsEnabled = true;
    elbowLeft.overrideSoftLimitsEnable(true);
  }

  public boolean getElbowLimitSwitch() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      return elbowLimitSwitch.get();
    } else {
      return false;
    }
  }

  public boolean isElbowZeroed() {
    return elbowZeroed;
  }

  public void beginElbowZeroSequence() {
    if (elbowEnabled && (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2)) {
      disableElbowLimits();
      elbowLimitsEnabled = false;
    }
  }

  public void disableElbow() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      elbowEnabled = false;
      elbowLeft.set(ControlMode.Disabled, 0);
    }
  }

  public void enableElbow() {
    if ((RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) && 
    (Robot.oi == null || Robot.oi.isArmEnabled())) {
      elbowEnabled = true;
      updateElbowSetpoint();
    }
  }
}
