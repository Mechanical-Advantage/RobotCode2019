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
  private static final double elbowOffsetHigh = 0; // Elbow offset applied when shoulder is raised
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

        elbowLeft.configContinuousCurrentLimit(elbowContinousCurrentLimit);
        elbowLeft.configPeakCurrentLimit(elbowPeakCurrentLimit);
        elbowLeft.configPeakCurrentDuration(elbowPeakCurrentLimitDuration);
        elbowLeft.enableCurrentLimit(elbowEnableCurrentLimit);
        elbowRight.configContinuousCurrentLimit(elbowContinousCurrentLimit);
        elbowRight.configPeakCurrentLimit(elbowPeakCurrentLimit);
        elbowRight.configPeakCurrentDuration(elbowPeakCurrentLimitDuration);
        elbowRight.enableCurrentLimit(elbowEnableCurrentLimit);
        wrist.configContinuousCurrentLimit(wristContinousCurrentLimit);
        wrist.configPeakCurrentLimit(wristPeakCurrentLimit);
        wrist.configPeakCurrentDuration(wristPeakCurrentLimitDuration);
        wrist.enableCurrentLimit(wristEnableCurrentLimit);
        telescope.configContinuousCurrentLimit(telescopeContinousCurrentLimit);
        telescope.configPeakCurrentLimit(telescopePeakCurrentLimit);
        telescope.configPeakCurrentDuration(telescopePeakCurrentLimitDuration);
        telescope.enableCurrentLimit(telescopeEnableCurrentLimit);

        setShoulderRaised(false); // Will set elbow limits as well and ensures consistent state
    }
  }

  @Override
  public void periodic() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      if (RobotMap.tuningMode) {
        initPID();
      }
      if (!elbowZeroed && getElbowLimitSwitch()) {
        setElbowZeroed();
      }
      if (!wristZeroed && getWristLimitSwitch()) {
        setWristZeroed();
      }
      elbowCurrentSchoolZone.applyPosition(getElbowPosition());
      wristSchoolZone.applyPosition(getRelativeWristPosition());
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
      updateWristSetpoint();
    }
  }

  public boolean getWristLimitSwitch() {
    // TODO add this
    return false;
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
