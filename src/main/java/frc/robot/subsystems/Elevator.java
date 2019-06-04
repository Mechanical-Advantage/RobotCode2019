/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.RobotMap.RobotType;
import frc.robot.util.SchoolZone;

/**
 * This is the elvator subsystem for ROBOT_REBOT
 */
public class Elevator extends Subsystem {
  private static final boolean elevatorMasterReversed = true;
  private static final boolean elevatorSlaveReversed = true;
  private static final NeutralMode neutralMode = NeutralMode.Coast;
  private static final double schoolZoneSpeedLimit = 0.1;
  private static final double schoolZoneLowerStart = 0;
  private static final double schoolZoneUpperStart = 0;
  private static final double peakOutput = 0.5;
  public static final double startingPosition = 0;

  private static final boolean enableCurrentLimit = false;
  private static final int continuousCurrentLimit = 0;
  private static final int peakCurrentLimit = 0;
  private static final int peakCurrentLimitDuration = 0; // ms

  private static final double elevatorUpperLimit = 0;
  private static final double elevatorLowerLimit = 0;
  private static final double schoolZoneUpperLimit = 0;
  private static final double schoolZoneLowerLimit = 0;

  // private static final TunableNumber kPElevator = new
  // TunableNumber("Elevator/p");
  // private static final TunableNumber kIElevator = new
  // TunableNumber("Elevator/i");
  // private static final TunableNumber kDElevator = new
  // TunableNumber("Elevator/d");

  private TalonSRX elevatorMaster;
  private TalonSRX elevatorSlave;

  private double tagetPosition;
  private SchoolZone schoolZone;
  private SchoolZone currentSchoolZone;

  private boolean elevatorEnabled;
  private boolean elevatorLimitsEnabled = true;
  private boolean elevatorOpenLoop = true;
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

      // schoolZone.setControllerLimits(); // This sets the peak output of the
      // controllers

      elevatorMaster.configContinuousCurrentLimit(continuousCurrentLimit);
      elevatorMaster.configPeakCurrentLimit(peakCurrentLimit);
      elevatorMaster.configPeakCurrentDuration(peakCurrentLimitDuration);
      elevatorMaster.enableCurrentLimit(enableCurrentLimit);

      elevatorSlave.configFactoryDefault();
      elevatorSlave.setInverted(elevatorSlaveReversed);
      elevatorSlave.setNeutralMode(neutralMode);

      elevatorSlave.follow(elevatorMaster);
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

  public void stop() {
    if (available()) {
      elevatorMaster.neutralOutput();
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
