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
import frc.robot.RobotMap;
import frc.robot.SchoolZone;
import frc.robot.RobotMap.RobotType;
import frc.robot.TunableNumber;

/**
 * 2019 robot arm
 */
public class Arm extends Subsystem {

  private static final FeedbackDevice elbowSensorType = FeedbackDevice.CTRE_MagEncoder_Relative;
  private static final int elbowTicksPerRotation = 4096;
  private static final boolean elbowSensorLeftReversed = false;
  private static final boolean elbowSensorRightReversed = false;
  private static final boolean elbowOutputLeftReversed = false;
  private static final boolean elbowOutputRightReversed = false;
  private static final boolean elbowDiffPIDPolarity = false; // Swaps syncronization correction
  private static final double elbowZeroedPosition = 0; // deg
  private static final double elbowLowerLimitLow = 0;
  private static final double elbowUpperLimitLow = 360;
  private static final double elbowLowerLimitHigh = 0;
  private static final double elbowUpperLimitHigh = 360;
  private static final int elbowReduction = 1; // Multiplier on setpoints
  private static final double elbowOffsetLow = 0; // Elbow offset applied when shoulder is lowered
  private static final double elbowOffsetHigh = 60; // Elbow offset applied when shoulder is raised
  private static final double elbowSchoolZoneSpeedLimit = 0.2;
  private static final double elbowLowSchoolZoneLowerStart = -360;
  private static final double elbowLowSchoolZoneUpperStart = 360;
  private static final double elbowHighSchoolZoneLowerStart = -360;
  private static final double elbowHighSchoolZoneUpperStart = 360;

  private static final FeedbackDevice wristSensorType = FeedbackDevice.CTRE_MagEncoder_Relative;
  private static final boolean wristSensorReversed = false;
  private static final boolean wristOutputReversed = false;
  private static final double wristZeroedPosition = 0; // Wrist position 0 should be same as elbow direction
  private static final int wristTicksPerRotation = 4096;
  private static final int wristReduction = 1;
  private static final double wristOffsetLow = 0;
  private static final double wristOffsetHigh = 0;
  private static final double wristElbowRatio = 33; // How much by divide elbow pos by to determine effect on wrist
  private static final double wristLowerLimit = -180;
  private static final double wristUpperLimit = 180;
  private static final double wristSchoolZoneSpeedLimit = 0.2;
  private static final double wristSchoolZoneLowerStart = -180;
  private static final double wristSchoolZoneUpperStart = 180;

  private static final FeedbackDevice telescopeSensorType = FeedbackDevice.CTRE_MagEncoder_Relative;
  private static final boolean telescopeSensorReversed = false;
  private static final boolean telescopeOutputReversed = false;
  private static final int telescopeTicksPerRotation = 4096;
  private static final int telescopeReduction = 1;
  private static final double telescopeInchesPerRotation = 1;
  private static final double telescopeZeroedPosition = 0;
  private static final int telescopeZeroAmps = 40; // How many amps the threshold for considering the telescope to have zeroed is
  private static final double telescopeMaxExtension = 0; // How far the telescope can extend
  private static final double telescopeSchoolZoneSpeedLimit = 0.2;
  private static final double telescopeSchoolZoneLowerStart = 2;
  private static final double telescopeSchoolZoneUpperStart = telescopeMaxExtension - 2;

  private static final double allowedFrameExtension = 30;
  private static final double bicepLength = 0;
  private static final double forearmLength = 0; // Length of the forearm with no telescope extension
  private static final double framePerimeterFrontFromShoulder = 6; // How far forward the front edge of the frame perimeter is from the shoulder joint
  private static final double framePerimeterBackFromShoulder = -20; // How far forward the back edge of the frame perimeter is from the shoulder joint

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
  private static final double shoulderDistanceLow = -1 * 
    Math.cos(Math.toRadians(elbowOffsetLow)) * bicepLength; // How far forward of the shoulder base the elbow joint is when the shoulder is lowered
  private static final double shoulderDistanceHigh = -1 * 
    Math.cos(Math.toRadians(elbowOffsetHigh)) * bicepLength; // How far forward of the shoulder base the elbow joint is when the shoulder is raised
  private static final int telescopeMaxExtensionTicks = convertTelescopeInchesToTicks(telescopeMaxExtension);

  private TalonSRX elbowLeft;
  private TalonSRX elbowRight;
  private TalonSRX wrist;
  private TalonSRX telescope;
  private DoubleSolenoid shoulder1;
  private DoubleSolenoid shoulder2;
  private DigitalInput elbowLimitSwitch;

  private boolean shoulderRaised = false;
  private double targetElbowPosition;
  private boolean elbowZeroed = false;
  private SchoolZone elbowLowSchoolZone;
  private SchoolZone elbowHighSchoolZone;
  private SchoolZone elbowCurrentSchoolZone;
  private boolean wristZeroed = false;
  private WristPosition targetWristPosition;
  private SchoolZone wristSchoolZone;
  private boolean telescopeZeroed = false;
  private int previousTelescopeLimit;
  private double targetTelescopePosition;
  private int targetTelescopePositionTicks;
  private SchoolZone telescopeSchoolZone;

  /*
  Note: We decided wrist should not use remote elbow sensor and setpoint should be updated when elbow target changed based on new target.
  Limits can then be set
  This calculation (elbow wrist adjustment) should be based on zeros
  Syncing elbow talons:
    Option 1 (what code has now): PIDs run on left side, primary drives to setpoint, aux drives diff to 0, right is an auxOutput follower
    Option 2: Each side tries to drive to setpoint and has aux PID to minimize difference
    Option 2 does not guarantee equal response but might correct for differences better
  For option 1, can a coefficient on the "slave" encoder act like the Spork feed forwards (maybe not)
  */

  public Arm() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || 
      RobotMap.robot == RobotType.ROBOT_2019_2) {
        kPElbow.setDefault(0);
        kIElbow.setDefault(0);
        kDElbow.setDefault(0);
        kPElbowSync.setDefault(0);
        kIElbowSync.setDefault(0);
        kDElbowSync.setDefault(0);
        kPWrist.setDefault(0);
        kIWrist.setDefault(0);
        kDWrist.setDefault(0);
        kPTelescope.setDefault(0);
        kITelescope.setDefault(0);
        kDTelescope.setDefault(0);

        shoulder1 = new DoubleSolenoid(RobotMap.armShoulder1PCM, 
        RobotMap.armShoulder1Extend, RobotMap.armShoulder1Retract);
        shoulder2 = new DoubleSolenoid(RobotMap.armShoulder2PCM, 
        RobotMap.armShoulder2Extend, RobotMap.armShoulder2Retract);

        elbowLimitSwitch = new DigitalInput(RobotMap.armElbowLimitSwitch);

        elbowLeft = new TalonSRX(RobotMap.armElbowLeft);
        elbowRight = new TalonSRX(RobotMap.armElbowRight);
        wrist = new TalonSRX(RobotMap.armWrist);
        telescope = new TalonSRX(RobotMap.armTelescope);

        elbowLowSchoolZone = new SchoolZone(elbowSchoolZoneSpeedLimit, 
          elbowLowSchoolZoneLowerStart, elbowLowSchoolZoneUpperStart, 
          elbowLeft, elbowRight);
        elbowHighSchoolZone = new SchoolZone(elbowSchoolZoneSpeedLimit, 
          elbowHighSchoolZoneLowerStart, elbowHighSchoolZoneUpperStart, 
          elbowLeft, elbowRight);
        wristSchoolZone = new SchoolZone(wristSchoolZoneSpeedLimit, 
          wristSchoolZoneLowerStart, wristSchoolZoneUpperStart, 
          wrist);
        telescopeSchoolZone = new SchoolZone(telescopeSchoolZoneSpeedLimit, 
          telescopeSchoolZoneLowerStart, telescopeSchoolZoneUpperStart, 
          telescope);

        elbowLeft.configSelectedFeedbackSensor(elbowSensorType);
        elbowLeft.setSensorPhase(elbowSensorLeftReversed);
        elbowLeft.setInverted(elbowOutputLeftReversed);
        elbowRight.configSelectedFeedbackSensor(elbowSensorType);
        elbowRight.setSensorPhase(elbowSensorRightReversed);
        elbowRight.setInverted(elbowOutputRightReversed);
        wrist.configSelectedFeedbackSensor(wristSensorType);
        wrist.setSensorPhase(wristSensorReversed);
        wrist.setInverted(wristOutputReversed);
        telescope.configSelectedFeedbackSensor(telescopeSensorType);
        telescope.setSensorPhase(telescopeSensorReversed);
        telescope.setInverted(telescopeOutputReversed);

        initPID();

        elbowLeft.configRemoteFeedbackFilter(RobotMap.armElbowRight, 
          RemoteSensorSource.TalonSRX_SelectedSensor, 0);
        elbowLeft.configSensorTerm(SensorTerm.Diff0, elbowSensorType);
        elbowLeft.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0);
        elbowLeft.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, 1, configTimeout);
        elbowLeft.selectProfileSlot(0, 0); // Use slot 0 for main PID
        elbowLeft.selectProfileSlot(1, 1); // Use slot 1 for secondary PID
        elbowLeft.configAuxPIDPolarity(elbowDiffPIDPolarity);
        elbowRight.follow(elbowLeft, FollowerType.AuxOutput1); // Aux PID get applied in opposite direction on right
        elbowLeft.configForwardSoftLimitEnable(true);
        elbowLeft.configReverseSoftLimitEnable(true);
        elbowRight.configForwardSoftLimitEnable(true);
        elbowRight.configReverseSoftLimitEnable(true);

        wrist.configForwardSoftLimitThreshold(convertWristRelativePositionToTicks(
          wristUpperLimit));
        wrist.configReverseSoftLimitThreshold(convertWristRelativePositionToTicks(
          wristLowerLimit));
        wrist.configForwardSoftLimitEnable(true);
        wrist.configReverseSoftLimitEnable(true);

        telescope.configReverseSoftLimitThreshold(0);
        telescope.configForwardSoftLimitEnable(true);
        telescope.configReverseSoftLimitEnable(true);

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

        setShoulderRaised(false); // Will set elbow limits as well and ensures consistent state
    }
  }

  @Override
  public void periodic() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      if (RobotMap.tuningMode) {
        initPID();
      }
      // Should zeroing multiple times be allowed?
      if (!elbowZeroed && getElbowLimitSwitch()) {
        setElbowZeroed();
      } else {
        elbowCurrentSchoolZone.applyPosition(getElbowPosition());
      }
      if (!wristZeroed && getWristLimitSwitch()) {
        setWristZeroed();
      } else {
        wristSchoolZone.applyPosition(getRelativeWristPosition());
      }
      if (!telescopeZeroed && getTelescopeLimitSensed()) {
        setTelescopeZeroed();
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
      shoulderRaised = raise;
      shoulder1.set(raise ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
      shoulder2.set(raise ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
      // May need to update setpoints for other parts here
      updateElbowSetpoint();
      updateElbowLimits();
      updateWristSetpoint();
    }
  }
  public boolean isShoulderRaised() {
    return shoulderRaised;
  }

  /**
   * Set the elbow position
   * @param position How many degrees from forward parallel to the floor
   */
  public void setElbowPosition(double position) {
    targetElbowPosition = position;
    updateElbowSetpoint();
    updateWristSetpoint();
  }
  /**
   * Get the target position of the elbow
   * @return The target elbow position in degrees from floor
   */
  public double getElbowTargetPosition() {
    return targetElbowPosition;
  }

  /**
   * Get the actual position of the elbow
   * @return The elbow position in degrees from floor
   */
  public double getElbowPosition() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      double position = (elbowLeft.getSelectedSensorPosition() + 
        elbowRight.getSelectedSensorPosition()) / 2; // Average position
      position = position / elbowReduction / elbowTicksPerRotation * 360; // Convert to degrees
      if (shoulderRaised) {
        position = position - elbowOffsetLow;
      } else {
        position = position - elbowOffsetHigh;
      }
      return position;
    } else {
      return 0;
    }
  }

  private void updateElbowSetpoint() {
    if (elbowZeroed && (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2)) {
      double setpoint;
      if (shoulderRaised) {
        setpoint = targetElbowPosition + elbowOffsetHigh;
      } else {
        setpoint = targetElbowPosition + elbowOffsetLow;
      }
      setpoint = setpoint / 360 * elbowTicksPerRotation; // Convert degrees to ticks
      setpoint *= elbowReduction;
      elbowLeft.set(ControlMode.Position, setpoint, DemandType.AuxPID, 0); // Aux PID (encoder difference) should try to be 0
    }
  }

  private void updateElbowLimits() {
    if(RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      double lowerLimit;
      double upperLimit;
      if (shoulderRaised) {
        lowerLimit = elbowLowerLimitHigh;
        upperLimit = elbowUpperLimitHigh;
        elbowCurrentSchoolZone = elbowHighSchoolZone;
      } else {
        lowerLimit = elbowLowerLimitLow;
        upperLimit = elbowUpperLimitLow;
        elbowCurrentSchoolZone = elbowLowSchoolZone;
      }
      // Ensure school zones are applied if needed
      elbowCurrentSchoolZone.applyPosition(getElbowPosition());
      elbowCurrentSchoolZone.setControllerLimits();
      int finalLowerLimit = (int)Math.round(lowerLimit / 360 * elbowTicksPerRotation 
        * elbowReduction);
      int finalUpperLimit = (int)Math.round(upperLimit / 360 * elbowTicksPerRotation 
        * elbowReduction);
      elbowLeft.configForwardSoftLimitThreshold(finalUpperLimit);
      elbowLeft.configReverseSoftLimitThreshold(finalLowerLimit);
      elbowRight.configForwardSoftLimitThreshold(finalUpperLimit);
      elbowRight.configReverseSoftLimitThreshold(finalLowerLimit);
    }
  }

  /**
   * Set the elbow to be zeroed
   */
  private void setElbowZeroed() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      int newPosition = (int)Math.round(elbowZeroedPosition / 360 * elbowTicksPerRotation 
        * elbowReduction);
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
      return elbowLimitSwitch.get();
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
    double newPosition = (double)position / wristReduction / wristTicksPerRotation * 360;
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
    if (shoulderRaised && fromFloor) {
      position += wristOffsetLow;
    } else if (fromFloor) {
      position += wristOffsetHigh;
    }
    return (int)Math.round(position / 360 * wristTicksPerRotation * wristReduction);
  }

  private void updateWristSetpoint() {
    if (wristZeroed && RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      int setpoint = convertWristPositionToTicks(targetWristPosition.getAngle(), 
        true, getElbowTargetPosition());
      // Make sure aux PID does nothing with DemandType.Neutral
      wrist.set(ControlMode.Position, setpoint, DemandType.Neutral, 0);
    }
  }

  private void setWristZeroed() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      wrist.setSelectedSensorPosition(convertWristRelativePositionToTicks(
        wristZeroedPosition));
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
    if (RobotMap.robot == RobotType.ROBOT_2019 || 
    RobotMap.robot == RobotType.ROBOT_2019_2) {
      return telescope.getOutputCurrent();
    }
    return 0;
  }

  public double getTelescopePosition() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || 
    RobotMap.robot == RobotType.ROBOT_2019_2) {
      return convertTelescopeTicksToInches(telescope.getSelectedSensorPosition());
    }
    return 0;
  }

  public double getTelescopeTargetPosition() {
    return targetTelescopePosition;
  }

  public void setTelescopePosition(double position) {
    if (RobotMap.robot == RobotType.ROBOT_2019 || 
    RobotMap.robot == RobotType.ROBOT_2019_2) {
      targetTelescopePosition = position;
      targetTelescopePositionTicks = convertTelescopeInchesToTicks(position);
      // Passing true to updateTelescopeForwardLimit forces the setpoint to be 
      // changed on the controller
      updateTelescopeForwardLimit(true);
    }
  }

  private static double convertTelescopeTicksToInches(int ticks) {
    return (double)ticks / telescopeReduction / telescopeTicksPerRotation * 
        telescopeInchesPerRotation;
  }

  private static int convertTelescopeInchesToTicks(double position) {
    return (int)Math.round(position / telescopeInchesPerRotation * 
      telescopeTicksPerRotation * telescopeReduction);
  }

  private void setTelescopeZeroed() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || 
    RobotMap.robot == RobotType.ROBOT_2019_2) {
      telescope.setSelectedSensorPosition(convertTelescopeInchesToTicks(
        telescopeZeroedPosition));
      telescopeZeroed = true;
      telescope.overrideSoftLimitsEnable(true);
    }
  }

  private boolean getTelescopeLimitSensed() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || 
     RobotMap.robot == RobotType.ROBOT_2019_2) {
      // If the motor is not moving backwards we can't possibly be at the limit
      return getTelescopeCurrent() >= telescopeZeroAmps && 
        telescope.getMotorOutputPercent() < 0;
    }
    return false;
  }

  /**
   * Updates the forward soft limit of the telescope and changes the setpoint if needed.
   * @param setpointChanged Whether the setpoint of the telescope has been changed and needs to be reevaluated
   */
  private void updateTelescopeForwardLimit(boolean setpointChanged) {
    double currentMaxExtension = getAllowedTelescopeExtension(getElbowPosition());
    int currentMaxExtensionTicks = convertTelescopeInchesToTicks(
      currentMaxExtension);
    int limit = currentMaxExtensionTicks > telescopeMaxExtensionTicks ?
      telescopeMaxExtensionTicks : currentMaxExtensionTicks;
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

  /**
   * Calculates how far outside the frame perimeter the arm currently is
   * @return Extension outside the frame perimeter (inches)
   */
  public double getDistanceOutsideFrame() {
    if (RobotMap.robot == RobotType.ROBOT_2019 ||
    RobotMap.robot == RobotType.ROBOT_2019_2) {
      double elbowPosition = getElbowPosition();
      double distanceFromElbow = Math.cos(Math.toRadians(elbowPosition)) *
        (getTelescopePosition() + forearmLength);
      double distanceFromShoulder = distanceFromElbow + (isShoulderRaised() ?
        shoulderDistanceHigh : shoulderDistanceLow);
      double frameExtension = distanceFromShoulder - (elbowPosition <= 90 ? 
        framePerimeterFrontFromShoulder : framePerimeterBackFromShoulder);
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
   * @param elbowPosition The angle of the elbow (0 = parallel with floor in front)
   * @return Allowed telescope extension (inches)
   */
  private double getAllowedTelescopeExtension(double elbowPosition) {
    if (RobotMap.robot == RobotType.ROBOT_2019 ||
    RobotMap.robot == RobotType.ROBOT_2019_2) {
      double allowedDistanceFromShoulder = allowedFrameExtension * 
        (elbowPosition > 90 ? -1 : 1) + (elbowPosition <= 90 ? 
        framePerimeterFrontFromShoulder : framePerimeterBackFromShoulder);
      double allowedDistanceFromElbow = allowedDistanceFromShoulder - 
        (isShoulderRaised() ? shoulderDistanceHigh : shoulderDistanceLow);
      double allowedExtensionFromElbow = allowedDistanceFromElbow / 
        Math.cos(Math.toRadians(elbowPosition));
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
