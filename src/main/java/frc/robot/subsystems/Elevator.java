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
  private static final int elevatorMasterTicksPerRotation = 4096;
  private static final double elevatorMasterDistancePerRotation = 2.5; // inches
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
  public static final double allowableError = 2;

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

  private boolean schoolZonesEnabled = true;
  private double targetPosition;
  private boolean openLoop;

  private static final FeedbackDevice encoderType = FeedbackDevice.CTRE_MagEncoder_Relative;
  private static final int configTimeout = 0;
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

      // elevatorMaster.configForwardSoftLimitThreshold((int) upperLimit,
      // configTimeout);
      // elevatorMaster.configReverseSoftLimitThreshold((int) lowerLimit,
      // configTimeout);
      // elevatorMaster.configForwardSoftLimitEnable(true, configTimeout);
      // elevatorMaster.configReverseSoftLimitEnable(true, configTimeout);

      elevatorMaster.configAllowableClosedloopError(0,
          (int) (allowableError / elevatorMasterDistancePerRotation * elevatorMasterTicksPerRotation), configTimeout);
      elevatorMaster.configMotionCruiseVelocity(
          (int) (motMagCruiseVelocity / elevatorMasterDistancePerRotation * elevatorMasterTicksPerRotation),
          configTimeout);
      elevatorMaster.configMotionAcceleration(
          (int) (motMagAccel / elevatorMasterDistancePerRotation * elevatorMasterTicksPerRotation), configTimeout);
      elevatorMaster.selectProfileSlot(0, 0);
      elevatorMaster.configSelectedFeedbackSensor(encoderType, 0, configTimeout);
      elevatorMaster.setSelectedSensorPosition(startingPosition);
      targetPosition = elevatorMaster.getSelectedSensorPosition();

      elevatorSlave.configFactoryDefault();
      elevatorSlave.setInverted(elevatorSlaveReversed);
      elevatorSlave.setNeutralMode(neutralMode);

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
      position = position / elevatorMasterTicksPerRotation * elevatorMasterDistancePerRotation;
      return position;
    } else {
      return 0;
    }
  }

  public void periodic() {
    if (schoolZonesEnabled) {
      schoolZone.applyPosition(getElevatorPosition());
    }
    if (elevatorMaster.getSensorCollection().isRevLimitSwitchClosed()) {
      elevatorMaster.setSelectedSensorPosition(
          (int) (lowerLimit / elevatorMasterDistancePerRotation * elevatorMasterTicksPerRotation));
    }
    if (elevatorMaster.getSensorCollection().isFwdLimitSwitchClosed()) {
      elevatorMaster.setSelectedSensorPosition(
          (int) (upperLimit / elevatorMasterDistancePerRotation * elevatorMasterTicksPerRotation));
    }
    initPID();
    SmartDashboard.putNumber("Elevator Position", getElevatorPosition());
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
      this.targetPosition = position / elevatorMasterDistancePerRotation * elevatorMasterTicksPerRotation;
      elevatorMaster.set(useMotionMagic ? ControlMode.MotionMagic : ControlMode.Position, targetPosition,
          DemandType.Neutral, 0);
    }
  }

  public void adjustTarget(double change) {
    if (available()) {
      openLoop = false;
      double newTarget = (targetPosition / elevatorMasterTicksPerRotation * elevatorMasterDistancePerRotation) + change;
      newTarget = newTarget < lowerLimit ? lowerLimit : newTarget;
      newTarget = newTarget > upperLimit ? upperLimit : newTarget;
      this.targetPosition = newTarget / elevatorMasterDistancePerRotation * elevatorMasterTicksPerRotation;
      elevatorMaster.set(ControlMode.Position, targetPosition, DemandType.Neutral, 0);
    }
  }

  public void holdTarget() {
    if (available()) {
      openLoop = false;
      elevatorMaster.set(ControlMode.Position, targetPosition, DemandType.Neutral, 0);
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ReBotRunElevatorWithJoystick());
  }
}
