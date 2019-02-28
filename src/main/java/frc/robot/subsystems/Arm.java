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
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.util.SchoolZone;
import frc.robot.RobotMap.RobotType;
import frc.robot.util.TunableNumber;

/**
 * 2019 robot arm
 */
public class Arm extends Subsystem {

  private static final FeedbackDevice elbowSensorType = FeedbackDevice.CTRE_MagEncoder_Relative;
  private static final int elbowTicksPerRotation = 4096;
  private static final boolean elbowSensorLeftReversed = true;
  private static final boolean elbowSensorRightReversed = true;
  private static final boolean elbowOutputLeftReversed = false;
  private static final boolean elbowOutputRightReversed = true;
  private static final boolean elbowDiffPIDPolarity = true; // Swaps syncronization correction
  private static final boolean elbowUseMotionMagic = false;
  private static final double elbowMotMagAccel = 20;
  private static final double elbowMotMagCruiseVelocity = 50;
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
  private static final double elbowAllowableErrorSync = 2;
  private static final double elbowZeroPercent = /*-0.05*/0;

  private static final FeedbackDevice wristSensorType = FeedbackDevice.CTRE_MagEncoder_Relative;
  private static final boolean wristSensorReversed = false;
  private static final boolean wristOutputReversed = false;
  private static final double wristZeroedPosition = 0; // Wrist position 0 should be same as elbow direction
  private static final int wristTicksPerRotation = 4096;
  private static final double wristReduction = (60/22)*33*1;
  private static final double wristOffsetLow = 0;
  private static final double wristOffsetHigh = 60;
  private static final double wristElbowRatio = 33; // How much by divide elbow pos by to determine effect on wrist
  private static final double wristLowerLimit = -180;
  private static final double wristUpperLimit = 180;
  private static final double wristSchoolZoneSpeedLimit = 0.2;
  private static final double wristSchoolZoneLowerStart = -180;
  private static final double wristSchoolZoneUpperStart = 180;
  private static final double wristPeakOutput = /* 1 */0;
  private static final double wristNominalOutputForward = 0;
  private static final double wristNominalOutputReverse = 0;
  private static final double wristAllowableError = 0;
  private static final double wristZeroPercent = /*-0.05*/0;

  private static final FeedbackDevice telescopeSensorType = FeedbackDevice.CTRE_MagEncoder_Relative;
  private static final boolean telescopeSensorReversed = false;
  private static final boolean telescopeOutputReversed = false;
  private static final int telescopeTicksPerRotation = 4096;
  private static final double telescopeReduction = (60/22)*33*(36/24);
  private static final double telescopeInchesPerRotation = 1.432*Math.PI;
  private static final double telescopeZeroedPosition = 0;
  private static final double telescopeMaxExtension = 20; // How far the telescope can extend
  private static final double telescopeSchoolZoneSpeedLimit = 0.2;
  private static final double telescopeSchoolZoneLowerStart = 2;
  private static final double telescopeSchoolZoneUpperStart = telescopeMaxExtension - 2;
  private static final double telescopePeakOutput = /* 1 */0;
  private static final double telescopeNominalOutputForward = 0;
  private static final double telescopeNominalOutputReverse = 0;
  private static final double telescopeAllowableError = 0;
  private static final double telescopeZeroPercent = /*-0.05*/0;

  private static final double allowedFrameExtension = 30;
  private static final double bicepLength = 25;
  // Length of the forearm with no telescope extension
  private static final double forearmLength = 30.5;
  // How far forward the front edge of the frame perimeter is from the shoulder
  // joint
  private static final double framePerimeterFrontFromShoulder = 30;
  // How far forward the back edge of the frame perimeter is from the shoulder
  // joint
  private static final double framePerimeterBackFromShoulder = -1;

  private static final TunableNumber kPElbow = new TunableNumber("Arm Elbow/p");
  private static final TunableNumber kIElbow = new TunableNumber("Arm Elbow/i");
  private static final TunableNumber kDElbow = new TunableNumber("Arm Elbow/d");
  private static final TunableNumber kPElbowSync = new TunableNumber("Arm Elbow Sync/p");
  private static final TunableNumber kIElbowSync = new TunableNumber("Arm Elbow Sync/i");
  private static final TunableNumber kDElbowSync = new TunableNumber("Arm Elbow Sync/d");
  private static final TunableNumber kPWrist = new TunableNumber("Arm Wrist/p");
  private static final TunableNumber kIWrist = new TunableNumber("Arm Wrist/i");
  private static final TunableNumber kDWrist = new TunableNumber("Arm Wrist/d");
  private static final TunableNumber kPTelescope = new TunableNumber("Arm Telescope/p");
  private static final TunableNumber kITelescope = new TunableNumber("Arm Telescope/i");
  private static final TunableNumber kDTelescope = new TunableNumber("Arm Telescope/d");

  private static final boolean elbowEnableCurrentLimit = false;
  private static final int elbowContinousCurrentLimit = 0;
  private static final int elbowPeakCurrentLimit = 0;
  private static final int elbowPeakCurrentLimitDuration = 0; // ms
  private static final boolean wristEnableCurrentLimit = false;
  private static final int wristContinousCurrentLimit = 0;
  private static final int wristPeakCurrentLimit = 0;
  private static final int wristPeakCurrentLimitDuration = 0;
  private static final boolean telescopeEnableCurrentLimit = false;
  private static final int telescopeContinousCurrentLimit = 0;
  private static final int telescopePeakCurrentLimit = 0;
  private static final int telescopePeakCurrentLimitDuration = 0;

  private static final int configTimeout = 10; // Method forms without timeout used where possible

  // Calculated constants

  // How far forward of the shoulder base the elbow joint is when the shoulder is
  // lowered
  private static final double shoulderDistanceLow = -1 * Math.cos(Math.toRadians(elbowOffsetLow)) * bicepLength;
  // How far forward of the shoulder base the elbow joint is when the shoulder is
  // raised
  private static final double shoulderDistanceHigh = -1 * Math.cos(Math.toRadians(elbowOffsetHigh)) * bicepLength;
  private static final int telescopeMaxExtensionTicks = convertTelescopeInchesToTicks(telescopeMaxExtension);

  private TalonSRX elbowLeft;
  private TalonSRX elbowRight;
  private TalonSRX wrist;
  private TalonSRX telescope;
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
  private boolean wristZeroed = false;
  private WristPosition targetWristPosition;
  private SchoolZone wristSchoolZone;
  private boolean wristEnabled;
  private boolean telescopeZeroed = false;
  private int previousTelescopeLimit;
  private double targetTelescopePosition;
  private int targetTelescopePositionTicks;
  private SchoolZone telescopeSchoolZone;
  private boolean telescopeEnabled;

  /*
   * Note: We decided wrist should not use remote elbow sensor and setpoint should
   * be updated when elbow target changed based on new target. Limits can then be
   * set.
   * 
   * This calculation (elbow wrist adjustment) should be based on zeros
   * 
   * Syncing elbow talons:
   * 
   * Option 1 (what code has now): PIDs run on left side, primary drives to
   * setpoint, aux drives diff to 0, right is an auxOutput follower
   * 
   * Option 2: Each side tries to drive to setpoint and has aux PID to minimize
   * difference.
   * 
   * Option 2 does not guarantee equal response but might correct for differences
   * better.
   * 
   * For option 1, can a coefficient on the "slave" encoder act like the Spork
   * feed forwards (maybe not)
   */

  public Arm() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      // Elbow numbers sync still does not stop well
      kPElbow.setDefault(1.3);
      kIElbow.setDefault(0);
      kDElbow.setDefault(13);
      kPElbowSync.setDefault(0.5); // Some oscillation
      kIElbowSync.setDefault(0);
      kDElbowSync.setDefault(5); // Have tried 0, 5, 15
      kPWrist.setDefault(0);
      kIWrist.setDefault(0);
      kDWrist.setDefault(0);
      kPTelescope.setDefault(0);
      kITelescope.setDefault(0);
      kDTelescope.setDefault(0);

      shoulder1 = new DoubleSolenoid(RobotMap.armShoulder1PCM, RobotMap.armShoulder1Extend,
          RobotMap.armShoulder1Retract);
      shoulder2 = new DoubleSolenoid(RobotMap.armShoulder2PCM, RobotMap.armShoulder2Extend,
          RobotMap.armShoulder2Retract);

      elbowLimitSwitch = new DigitalInput(RobotMap.armElbowLimitSwitch);

      elbowLeft = new TalonSRX(RobotMap.armElbowLeft);
      elbowRight = new TalonSRX(RobotMap.armElbowRight);
      wrist = new TalonSRX(RobotMap.armWrist);
      telescope = new TalonSRX(RobotMap.armTelescope);

      elbowLowSchoolZone = new SchoolZone(elbowSchoolZoneSpeedLimit, elbowPeakOutput, elbowLowSchoolZoneLowerStart,
          elbowLowSchoolZoneUpperStart, elbowLeft, elbowRight);
      elbowHighSchoolZone = new SchoolZone(elbowSchoolZoneSpeedLimit, elbowPeakOutput, elbowHighSchoolZoneLowerStart,
          elbowHighSchoolZoneUpperStart, elbowLeft, elbowRight);
      wristSchoolZone = new SchoolZone(wristSchoolZoneSpeedLimit, wristPeakOutput, wristSchoolZoneLowerStart,
          wristSchoolZoneUpperStart, wrist);
      telescopeSchoolZone = new SchoolZone(telescopeSchoolZoneSpeedLimit, telescopePeakOutput,
          telescopeSchoolZoneLowerStart, telescopeSchoolZoneUpperStart, telescope);

      elbowLeft.configFactoryDefault();
      elbowLeft.configSelectedFeedbackSensor(elbowSensorType);
      elbowLeft.setSensorPhase(elbowSensorLeftReversed);
      elbowLeft.setInverted(elbowOutputLeftReversed);
      elbowRight.configFactoryDefault();
      elbowRight.configSelectedFeedbackSensor(elbowSensorType);
      elbowRight.setSensorPhase(elbowSensorRightReversed);
      elbowRight.setInverted(elbowOutputRightReversed);
      wrist.configFactoryDefault();
      wrist.configSelectedFeedbackSensor(wristSensorType);
      wrist.setSensorPhase(wristSensorReversed);
      wrist.setInverted(wristOutputReversed);
      telescope.configFactoryDefault();
      telescope.configSelectedFeedbackSensor(telescopeSensorType);
      telescope.setSensorPhase(telescopeSensorReversed);
      telescope.setInverted(telescopeOutputReversed);

      initPID();

      elbowLeft.configRemoteFeedbackFilter(RobotMap.armElbowRight, RemoteSensorSource.TalonSRX_SelectedSensor, 0);
      elbowLeft.configSensorTerm(SensorTerm.Sum0, elbowSensorType);
      elbowLeft.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.RemoteSensor0);
      elbowLeft.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, 1, configTimeout);
      elbowLeft.selectProfileSlot(0, 0); // Use slot 0 for main PID
      elbowLeft.selectProfileSlot(1, 1); // Use slot 1 for secondary PID
      elbowLeft.configAuxPIDPolarity(elbowDiffPIDPolarity);
      elbowLeft.configForwardSoftLimitEnable(true);
      elbowLeft.configReverseSoftLimitEnable(true);
      elbowLeft.configMotionCruiseVelocity(convertElbowPositionToTicks(elbowMotMagCruiseVelocity, false));
      elbowLeft.configMotionAcceleration(convertElbowPositionToTicks(elbowMotMagAccel, false));
      elbowLeft.configNominalOutputForward(elbowForwardNominalOutput);
      elbowLeft.configNominalOutputReverse(elbowReverseNominalOutput);
      elbowLeft.configAllowableClosedloopError(0, convertElbowPositionToTicks(
        elbowAllowableError, false));
      elbowLeft.configAllowableClosedloopError(1, convertElbowPositionToTicks(
        elbowAllowableErrorSync, false));
      elbowRight.configForwardSoftLimitEnable(true);
      elbowRight.configReverseSoftLimitEnable(true);
      elbowLeft.configNominalOutputForward(elbowForwardNominalOutput);
      elbowLeft.configNominalOutputReverse(elbowReverseNominalOutput);
      elbowLowSchoolZone.setControllerLimits(); // This sets the peak output of the controllers

      wrist.configForwardSoftLimitThreshold(convertWristRelativePositionToTicks(wristUpperLimit));
      wrist.configReverseSoftLimitThreshold(convertWristRelativePositionToTicks(wristLowerLimit));
      wrist.configNominalOutputForward(wristNominalOutputForward);
      wrist.configNominalOutputReverse(wristNominalOutputReverse);
      wrist.configAllowableClosedloopError(0, 
        convertWristRelativePositionToTicks(wristAllowableError));
      wrist.configForwardSoftLimitEnable(true);
      wrist.configReverseSoftLimitEnable(true);
      wristSchoolZone.setControllerLimits();

      telescope.configReverseSoftLimitThreshold(0);
      telescope.configNominalOutputForward(telescopeNominalOutputForward);
      telescope.configNominalOutputReverse(telescopeNominalOutputReverse);
      telescope.configAllowableClosedloopError(0, 
        convertTelescopeInchesToTicks(telescopeAllowableError));
      telescope.configForwardSoftLimitEnable(true);
      telescope.configReverseSoftLimitEnable(true);
      telescopeSchoolZone.setControllerLimits();

      elbowLeft.configContinuousCurrentLimit(elbowContinousCurrentLimit);
      elbowLeft.configPeakCurrentLimit(elbowPeakCurrentLimit);
      elbowLeft.configPeakCurrentDuration(elbowPeakCurrentLimitDuration);
      elbowLeft.enableCurrentLimit(elbowEnableCurrentLimit);
      elbowLeft.overrideSoftLimitsEnable(false); // Before zeroing this does not work
      elbowRight.configContinuousCurrentLimit(elbowContinousCurrentLimit);
      elbowRight.configPeakCurrentLimit(elbowPeakCurrentLimit);
      elbowRight.configPeakCurrentDuration(elbowPeakCurrentLimitDuration);
      elbowRight.enableCurrentLimit(elbowEnableCurrentLimit);
      elbowRight.overrideSoftLimitsEnable(false);
      wrist.configContinuousCurrentLimit(wristContinousCurrentLimit);
      wrist.configPeakCurrentLimit(wristPeakCurrentLimit);
      wrist.configPeakCurrentDuration(wristPeakCurrentLimitDuration);
      wrist.enableCurrentLimit(wristEnableCurrentLimit);
      wrist.overrideSoftLimitsEnable(false);
      telescope.configContinuousCurrentLimit(telescopeContinousCurrentLimit);
      telescope.configPeakCurrentLimit(telescopePeakCurrentLimit);
      telescope.configPeakCurrentDuration(telescopePeakCurrentLimitDuration);
      telescope.enableCurrentLimit(telescopeEnableCurrentLimit);
      telescope.overrideSoftLimitsEnable(false);

      // enableElbow();
      // enableWrist();
      // enableTelescope();
      disableElbow();
      disableWrist();
      disableTelescope();
      setShoulderRaised(false); // Will set elbow limits as well and ensures consistent state

      // Start zeroing the mechanisms
      // Normal control will not work until this is complete (see periodic())
      elbowLeft.set(ControlMode.PercentOutput, elbowZeroPercent, DemandType.Neutral, 0);
      wrist.set(ControlMode.PercentOutput, wristZeroPercent);
      telescope.set(ControlMode.PercentOutput, telescopeZeroPercent);
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
      if (!elbowZeroed) {
        if (getElbowLimitSwitch()) {
          setElbowZeroed();
        }
      } else {
        elbowCurrentSchoolZone.applyPosition(elbowPosition);
      }
      if (!wristZeroed) {
        if (getWristLimitSwitch()) {
          setWristZeroed();
        }
      } else {
        wristSchoolZone.applyPosition(getRelativeWristPosition());
      }
      if (!telescopeZeroed) {
        if (getTelescopeLimitSensed()) {
          setTelescopeZeroed();
        }
      } else {
        updateTelescopeForwardLimit(false);
        telescopeSchoolZone.applyPosition(getTelescopePosition());
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
    elbowLeft.config_kP(1, kPElbowSync.get());
    elbowLeft.config_kI(1, kIElbowSync.get());
    elbowLeft.config_kD(1, kDElbowSync.get());
    wrist.config_kP(0, kPWrist.get());
    wrist.config_kI(0, kIWrist.get());
    wrist.config_kD(0, kDWrist.get());
    telescope.config_kP(0, kPTelescope.get());
    telescope.config_kI(0, kITelescope.get());
    telescope.config_kD(0, kDTelescope.get());
  }

  public void setShoulderRaised(boolean raise) {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      targetShoulderRaised = raise;
      updateShoulderSetpoint();
      // May need to update setpoints for other parts here
      updateElbowLimits();
      updateElbowSetpoint();
      updateWristSetpoint();
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
    updateWristSetpoint();
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
      double position = (elbowLeft.getSelectedSensorPosition() + elbowRight.getSelectedSensorPosition()) / 2;
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
    if (elbowZeroed && (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2)
        && elbowEnabled) {
      double target;
      if (targetElbowPosition < getElbowLowerLimit(targetShoulderRaised)) {
        target = getElbowLowerLimit(targetShoulderRaised);
      } else if (targetElbowPosition > getElbowUpperLimit(targetShoulderRaised)) {
        target = getElbowUpperLimit(targetShoulderRaised);
      } else {
        target = targetElbowPosition;
      }
      // Aux PID (encoder difference) should try to be 0
      elbowLeft.set(elbowUseMotionMagic ? ControlMode.MotionMagic : ControlMode.Position,
          convertElbowPositionToTicks(target, true), DemandType.AuxPID, 0);
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
      elbowRight.configForwardSoftLimitThreshold(finalUpperLimit);
      elbowRight.configReverseSoftLimitThreshold(finalLowerLimit);
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
      elbowRight.setSelectedSensorPosition(newPosition);
      elbowZeroed = true;
      elbowLeft.overrideSoftLimitsEnable(true);
      elbowRight.overrideSoftLimitsEnable(true);
      updateElbowSetpoint();
      // May affect other parts
    }
  }

  public boolean getElbowLimitSwitch() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      // TODO undo this change
      return true;
      // return elbowLimitSwitch.get();
    } else {
      return false;
    }
  }

  public void setWristPosition(WristPosition position) {
    targetWristPosition = position;
    updateWristSetpoint();
  }

  public WristPosition getWristTargetPosition() {
    return targetWristPosition;
  }

  /**
   * Get the wrist position relative to the floor (0=flat, upright game piece)
   * 
   * @return The wrist position in degrees
   */
  public double getWristPosition() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      return convertWristTicksToPosition(wrist.getSelectedSensorPosition(0), true, getElbowPosition());
    } else {
      return 0;
    }
  }

  /**
   * Get the wrist position relative to the forearm
   * 
   * @return The wrist position in degrees
   */
  public double getRelativeWristPosition() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      // Don't apply offset here
      return convertWristTicksToRelativePosition(wrist.getSelectedSensorPosition(0));
    } else {
      return 0;
    }
  }

  private double convertWristTicksToRelativePosition(int position) {
    return convertWristTicksToPosition(position, false, -1);
  }

  private double convertWristTicksToPosition(int position, boolean fromFloor, double elbowPosition) {
    double newPosition = (double) position / wristReduction / wristTicksPerRotation * 360;
    if (shoulderRaised && fromFloor) {
      newPosition -= wristOffsetLow;
    } else if (fromFloor) {
      newPosition -= wristOffsetHigh;
    }
    if (fromFloor) {
      newPosition += elbowPosition / wristElbowRatio;
    }
    return newPosition;
  }

  private int convertWristRelativePositionToTicks(double position) {
    return convertWristPositionToTicks(position, false, -1);
  }

  private int convertWristPositionToTicks(double position, boolean fromFloor, double elbowPosition) {
    if (fromFloor) {
      position -= elbowPosition / wristElbowRatio;
    }
    if (targetShoulderRaised && fromFloor) {
      position += wristOffsetLow;
    } else if (fromFloor) {
      position += wristOffsetHigh;
    }
    return (int) Math.round(position / 360 * wristTicksPerRotation * wristReduction);
  }

  private void updateWristSetpoint() {
    if (wristZeroed && (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2)
        && wristEnabled) {
      int setpoint = convertWristPositionToTicks(targetWristPosition.getAngle(), true, getElbowTargetPosition());
      // Make sure aux PID does nothing with DemandType.Neutral
      wrist.set(ControlMode.Position, setpoint, DemandType.Neutral, 0);
    }
  }

  private void setWristZeroed() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      wrist.setSelectedSensorPosition(convertWristRelativePositionToTicks(wristZeroedPosition));
      wristZeroed = true;
      wrist.overrideSoftLimitsEnable(true);
      updateWristSetpoint();
    }
  }

  public boolean getWristLimitSwitch() {
    // TODO add this
    return false;
  }

  public double getTelescopeCurrent() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      return telescope.getOutputCurrent();
    }
    return 0;
  }

  public double getTelescopePosition() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      return convertTelescopeTicksToInches(telescope.getSelectedSensorPosition());
    }
    return 0;
  }

  public double getTelescopeTargetPosition() {
    return targetTelescopePosition;
  }

  public void setTelescopePosition(double position) {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      targetTelescopePosition = position;
      targetTelescopePositionTicks = convertTelescopeInchesToTicks(position);
      // Passing true to updateTelescopeForwardLimit forces the setpoint to be
      // changed on the controller
      updateTelescopeForwardLimit(true);
    }
  }

  private static double convertTelescopeTicksToInches(int ticks) {
    return (double) ticks / telescopeReduction / telescopeTicksPerRotation * telescopeInchesPerRotation;
  }

  private static int convertTelescopeInchesToTicks(double position) {
    return (int) Math.round(position / telescopeInchesPerRotation * telescopeTicksPerRotation * telescopeReduction);
  }

  private void setTelescopeZeroed() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      telescope.setSelectedSensorPosition(convertTelescopeInchesToTicks(telescopeZeroedPosition));
      telescopeZeroed = true;
      telescope.overrideSoftLimitsEnable(true);
    }
  }

  private boolean getTelescopeLimitSensed() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      return telescope.getSelectedSensorVelocity() == 0 && telescope.getMotorOutputPercent() < 0;
    }
    return false;
  }

  public void disableElbow() {
    elbowEnabled = false;
    elbowLeft.set(ControlMode.Disabled, 0);
    elbowRight.set(ControlMode.Disabled, 0);
  }

  public void disableWrist() {
    wristEnabled = false;
    wrist.set(ControlMode.Disabled, 0);
  }

  public void disableTelescope() {
    telescopeEnabled = false;
    telescope.set(ControlMode.Disabled, 0);
  }

  public void enableElbow() {
    if (Robot.oi == null || Robot.oi.isArmEnabled()) {
      elbowEnabled = true;
      elbowRight.follow(elbowLeft, FollowerType.AuxOutput1); // Aux PID get applied
      // in opposite direction on right
      updateElbowSetpoint();
    }
  }

  public void enableWrist() {
    if (Robot.oi == null || Robot.oi.isArmEnabled()) {
      wristEnabled = true;
      updateWristSetpoint();
    }
  }

  public void enableTelescope() {
    if (Robot.oi == null || Robot.oi.isArmEnabled()) {
      telescopeEnabled = true;
      updateTelescopeForwardLimit(true);
    }
  }

  /**
   * Updates the forward soft limit of the telescope and changes the setpoint if
   * needed.
   * 
   * @param setpointChanged Whether the setpoint of the telescope has been changed
   *                        and needs to be reevaluated
   */
  private void updateTelescopeForwardLimit(boolean setpointChanged) {
    if ((RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) && telescopeZeroed
        && elbowZeroed && telescopeEnabled) {
      double currentMaxExtension = getAllowedTelescopeExtension(getElbowPosition());
      int currentMaxExtensionTicks = convertTelescopeInchesToTicks(currentMaxExtension);
      int limit = currentMaxExtensionTicks > telescopeMaxExtensionTicks ? telescopeMaxExtensionTicks
          : currentMaxExtensionTicks;
      if (limit != previousTelescopeLimit || setpointChanged) {
        telescope.configForwardSoftLimitThreshold(limit, 0);
        if (targetTelescopePositionTicks > limit) {
          telescope.set(ControlMode.Position, limit);
        } else {
          telescope.set(ControlMode.Position, targetTelescopePositionTicks);
        }
        previousTelescopeLimit = limit;
      }
    }
  }

  /**
   * Calculates how far outside the frame perimeter the arm currently is
   * 
   * @return Extension outside the frame perimeter (inches)
   */
  public double getDistanceOutsideFrame() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      double elbowPosition = getElbowPosition();
      double distanceFromElbow = Math.cos(Math.toRadians(elbowPosition)) * (getTelescopePosition() + forearmLength);
      double distanceFromShoulder = distanceFromElbow
          + (isShoulderRaised() ? shoulderDistanceHigh : shoulderDistanceLow);
      double frameExtension = distanceFromShoulder
          - (elbowPosition <= 90 ? framePerimeterFrontFromShoulder : framePerimeterBackFromShoulder);
      if (elbowPosition > 90) {
        frameExtension *= -1;
      }
      return frameExtension;
    } else {
      return 0;
    }
  }

  /**
   * Calculates how much the telescope is allowed to extend at a given elbow angle
   * 
   * @param elbowPosition The angle of the elbow (0 = parallel with floor in
   *                      front)
   * @return Allowed telescope extension (inches)
   */
  private double getAllowedTelescopeExtension(double elbowPosition) {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      double allowedDistanceFromShoulder = allowedFrameExtension * (elbowPosition > 90 ? -1 : 1)
          + (elbowPosition <= 90 ? framePerimeterFrontFromShoulder : framePerimeterBackFromShoulder);
      double allowedDistanceFromElbow = allowedDistanceFromShoulder
          - (isShoulderRaised() ? shoulderDistanceHigh : shoulderDistanceLow);
      double allowedExtensionFromElbow = allowedDistanceFromElbow / Math.cos(Math.toRadians(elbowPosition));
      double allowedExtension = allowedExtensionFromElbow - forearmLength;
      return allowedExtension;
    } else {
      return 0;
    }
  }

  public enum WristPosition {
    UPRIGHT, FLAT, CARGO_PICKUP;

    private double getAngle() {
      switch (this) {
      case UPRIGHT:
        return 0;
      case FLAT:
        return 90;
      case CARGO_PICKUP:
        return 45;
      }
      return 0;
    }
  }

}
