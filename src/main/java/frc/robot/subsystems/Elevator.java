/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
  private static final double elevatorUpperLimit = 40; // TBD
  private static final double elevatorLowerLimit = 0;
  private static final double schoolZoneSpeedLimit = 0.2;
  private static final double schoolZoneLowerStart = 4;
  private static final double schoolZoneUpperStart = elevatorUpperLimit - 4;
  private static final double peakOutput = 1.0;
  public static final double startingPosition = 0;
  private static final double allowableError = 2;

  private static final boolean enableCurrentLimit = false;
  private static final int continuousCurrentLimit = 0;
  private static final int peakCurrentLimit = 0;
  private static final int peakCurrentLimitDuration = 0; // ms

  private static final TunableNumber kPElevator = new TunableNumber("Elevator/p");
  private static final TunableNumber kIElevator = new TunableNumber("Elevator/i");
  private static final TunableNumber kDElevator = new TunableNumber("Elevator/d");
  private static final TunableNumber kFElevator = new TunableNumber("Elevator/f");
  private static final TunableNumber kIZoneElevator = new TunableNumber("Elevator/izone");

  private TalonSRX elevatorMaster;
  private TalonSRX elevatorSlave;
  private SchoolZone schoolZone;
  private DigitalInput lowerLimit;
  private DigitalInput upperLimit;

  private boolean schoolZonesEnabled = true;
  private boolean openLoop = true;

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

      lowerLimit = new DigitalInput(RobotMap.elevatorLowLimitSwitch);
      upperLimit = new DigitalInput(RobotMap.elevatorHighLimitSwitch);

      elevatorMaster.configFactoryDefault();
      elevatorMaster.setInverted(elevatorMasterReversed);
      elevatorMaster.setNeutralMode(neutralMode);

      schoolZone.setControllerLimits(); // This sets the peak output of the
      // controllers

      elevatorMaster.configContinuousCurrentLimit(continuousCurrentLimit);
      elevatorMaster.configPeakCurrentLimit(peakCurrentLimit);
      elevatorMaster.configPeakCurrentDuration(peakCurrentLimitDuration);
      elevatorMaster.enableCurrentLimit(enableCurrentLimit);

      elevatorMaster.configForwardSoftLimitThreshold((int) elevatorUpperLimit, configTimeout);
      elevatorMaster.configReverseSoftLimitThreshold((int) elevatorLowerLimit, configTimeout);
      elevatorMaster.configForwardSoftLimitEnable(true, configTimeout);
      elevatorMaster.configReverseSoftLimitEnable(true, configTimeout);

      elevatorMaster.config_kP(0, kPElevator.get(), configTimeout);
      elevatorMaster.config_kI(0, kIElevator.get(), configTimeout);
      elevatorMaster.config_kD(0, kDElevator.get(), configTimeout);
      elevatorMaster.config_kF(0, kFElevator.get(), configTimeout);
      elevatorMaster.config_IntegralZone(0, (int) kIZoneElevator.get(), configTimeout);
      elevatorMaster.configAllowableClosedloopError(0,
          (int) (allowableError / elevatorMasterDistancePerRotation * elevatorMasterTicksPerRotation), configTimeout);
      elevatorMaster.selectProfileSlot(0, 0);

      elevatorSlave.configFactoryDefault();
      elevatorSlave.setInverted(elevatorSlaveReversed);
      elevatorSlave.setNeutralMode(neutralMode);

      elevatorSlave.follow(elevatorMaster);

      elevatorMaster.configSelectedFeedbackSensor(encoderType, 0, configTimeout);
    }
  }

  private boolean available() {
    return RobotMap.robot == RobotType.ROBOT_REBOT;
  }

  public void run(double power) {
    if (available()) {
      elevatorMaster.set(ControlMode.PercentOutput, power);
    }
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

  public void stop() {
    if (available()) {
      elevatorMaster.neutralOutput();
    }
  }

  public void periodic() {
    if (schoolZonesEnabled) {
      schoolZone.applyPosition(getElevatorPosition());
    }
    if (lowerLimit.get()) {
      elevatorMaster.setSelectedSensorPosition(
          (int) (elevatorLowerLimit / elevatorMasterDistancePerRotation * elevatorMasterTicksPerRotation));
    }
    if (upperLimit.get()) {
      elevatorMaster.setSelectedSensorPosition(
          (int) (elevatorUpperLimit / elevatorMasterDistancePerRotation * elevatorMasterTicksPerRotation));
    }
    SmartDashboard.putNumber("Elevator Position", getElevatorPosition());
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ReBotRunElevatorWithJoystick());
  }
}
