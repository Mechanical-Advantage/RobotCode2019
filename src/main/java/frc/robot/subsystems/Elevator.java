/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.RobotMap.RobotType;
import frc.robot.util.SchoolZone;
import frc.robot.util.TunableNumber;
import frc.robot.commands.ReBotRunElevatorWithJoystick;

/**
 * This is the elvator subsystem for ROBOT_REBOT
 */
public class Elevator extends Subsystem {
  private static final int ticksPerRotation = 4096;
  private static final double distancePerRotation = 2.5; // inches
  private static final boolean elevatorMasterReversed = true;
  private static final boolean elevatorSlaveReversed = false;
  private static final NeutralMode neutralMode = NeutralMode.Brake;
  private static final boolean useMotionMagic = true;
  private static final double motMagAccel = 1;
  private static final double motMagCruiseVelocity = 30;
  private static final double upperLimit = 40; // TBD
  private static final double lowerLimit = 0;
  private static final double schoolZoneSpeedLimit = 0.2;
  private static final double schoolZoneLowerStart = 4;
  private static final double schoolZoneUpperStart = upperLimit - 4;
  private static final double peakOutput = 1.0;
  public static final int startingPosition = 0;
  public static final double allowableError = 2; // in inches
  private static final boolean enableUpperSoftLimit = true;
  private static final boolean enableLowerSoftLimit = false;
  private static final boolean schoolZonesEnabled = true;
  private static final int configTimeout = 0;
  private static final FeedbackDevice encoderType = FeedbackDevice.CTRE_MagEncoder_Relative;

  private static final boolean enableCurrentLimit = false;
  private static final int continuousCurrentLimit = 0;
  private static final int peakCurrentLimit = 0;
  private static final int peakCurrentLimitDuration = 0; // ms

  private static final TunableNumber kPElevator = new TunableNumber("Elevator/p");
  private static final TunableNumber kIElevator = new TunableNumber("Elevator/i");
  private static final TunableNumber kDElevator = new TunableNumber("Elevator/d");

  private TalonSRX elevatorMaster;
  private TalonSRX elevatorSlave;
  private SchoolZone schoolZone;

  private double targetPosition; // in inches
  private boolean openLoop;

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public Elevator() {
    if (available()) {
      schoolZone = new SchoolZone(schoolZoneSpeedLimit, peakOutput, schoolZoneLowerStart, schoolZoneUpperStart,
          elevatorMaster);

      elevatorMaster = new TalonSRX(RobotMap.elevatorMaster);
      elevatorSlave = new TalonSRX(RobotMap.elevatorSlave);

      elevatorMaster.configFactoryDefault();
      elevatorMaster.setInverted(elevatorMasterReversed);
      elevatorMaster.setNeutralMode(neutralMode);

      schoolZone.setControllerLimits(); // This sets the peak output of the
      // controllers
      initPID();

      elevatorMaster.configContinuousCurrentLimit(continuousCurrentLimit);
      elevatorMaster.configPeakCurrentLimit(peakCurrentLimit);
      elevatorMaster.configPeakCurrentDuration(peakCurrentLimitDuration);
      elevatorMaster.enableCurrentLimit(enableCurrentLimit);

      elevatorMaster.configForwardSoftLimitThreshold(convertDistanceToTicks(upperLimit), configTimeout);
      elevatorMaster.configReverseSoftLimitThreshold(convertDistanceToTicks(lowerLimit), configTimeout);
      elevatorMaster.configForwardSoftLimitEnable(enableUpperSoftLimit, configTimeout);
      elevatorMaster.configReverseSoftLimitEnable(enableLowerSoftLimit, configTimeout);
      elevatorMaster.configSetParameter(ParamEnum.eClearPositionOnLimitR, 1, 0, 0);

      elevatorMaster.configAllowableClosedloopError(0, convertDistanceToTicks(allowableError), configTimeout);
      elevatorMaster.configMotionCruiseVelocity(convertDistanceToTicks(motMagCruiseVelocity), configTimeout);
      elevatorMaster.configMotionAcceleration(convertDistanceToTicks(motMagAccel), configTimeout);
      elevatorMaster.selectProfileSlot(0, 0);
      elevatorMaster.configSelectedFeedbackSensor(encoderType, 0, configTimeout);
      elevatorMaster.setSelectedSensorPosition(startingPosition);
      targetPosition = convertTicksToDistance(elevatorMaster.getSelectedSensorPosition());

      elevatorSlave.configFactoryDefault();
      elevatorSlave.setInverted(elevatorSlaveReversed);
      elevatorSlave.setNeutralMode(neutralMode);

      elevatorSlave.configContinuousCurrentLimit(continuousCurrentLimit);
      elevatorSlave.configPeakCurrentLimit(peakCurrentLimit);
      elevatorSlave.configPeakCurrentDuration(peakCurrentLimitDuration);
      elevatorSlave.enableCurrentLimit(enableCurrentLimit);

      elevatorSlave.follow(elevatorMaster);

    }
  }

  private boolean available() {
    return RobotMap.robot == RobotType.ROBOT_REBOT;
  }

  private void initPID() {
    elevatorMaster.config_kP(0, kPElevator.get(), configTimeout);
    elevatorMaster.config_kI(0, kIElevator.get(), configTimeout);
    elevatorMaster.config_kD(0, kDElevator.get(), configTimeout);
  }

  public double getElevatorPosition() {
    if (available()) {
      double position = elevatorMaster.getSelectedSensorPosition();
      return convertTicksToDistance(position);
    } else {
      return 0;
    }
  }

  public double getTargetPosition() {
    if (available()) {
      return targetPosition;
    } else {
      return 0;
    }
  }

  public void periodic() {
    if (available()) {
      if (schoolZonesEnabled) {
        schoolZone.applyPosition(getElevatorPosition());
      }
      initPID();
      SmartDashboard.putNumber("Elevator Position", getElevatorPosition());
    }
  }

  public void run(double power) {
    if (available()) {
      openLoop = true;
      elevatorMaster.set(ControlMode.PercentOutput, power);
    }
  }

  public void stop() {
    if (available()) {
      openLoop = true;
      elevatorMaster.neutralOutput();
    }
  }

  public void moveToPosition(double position) {
    if (available()) {
      openLoop = false;
      position = position < lowerLimit ? lowerLimit : position;
      position = position > upperLimit ? upperLimit : position;
      this.targetPosition = position;
      elevatorMaster.set(useMotionMagic ? ControlMode.MotionMagic : ControlMode.Position,
          convertDistanceToTicks(targetPosition), DemandType.Neutral, 0);
    }
  }

  public int convertDistanceToTicks(double distance) {
    return (int) (distance / distancePerRotation * ticksPerRotation);
  }

  public double convertTicksToDistance(double ticks) {
    return ticks / ticksPerRotation * distancePerRotation;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ReBotRunElevatorWithJoystick());
  }
}
