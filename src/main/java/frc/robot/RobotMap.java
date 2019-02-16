/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
  public static int rightMaster;
  public static int rightSlave;
  public static int rightSlave2;
  public static int leftMaster;
  public static int leftSlave;
  public static int leftSlave2;
  public static int vacuumMotor;
  public static int leftDriveGearSolenoid1;
  public static int leftDriveGearSolenoid2;
  public static int leftDriveGearPCM;
  public static int rightDriveGearSolenoid1;
  public static int rightDriveGearSolenoid2;
  public static int rightDriveGearPCM;
  public static double robotWidth;
  public static double robotLength;
  public static final boolean tuningMode = true;
  public static final RobotType robot = RobotType.EVERYBOT_2019;
  public static int minVelocityLow; // lower values will be treated as this value, RPM
  public static int maxVelocityLow; // maximum velocity when sticks are fully forward (value of 1), RPM
  public static int maxVelocityHigh;
  public static int minVelocityHigh;
  public static int maxAcceleration;
  public static int armElbowLeft;
  public static int armElbowRight;
  public static int armElbowLimitSwitch;
  public static int armWrist;
  public static int armTelescope;
  public static int armShoulder1Extend;
  public static int armShoulder1Retract;
  public static int armShoulder1PCM;
  public static int armShoulder2Extend;
  public static int armShoulder2Retract;
  public static int armShoulder2PCM;
  public static int intakeLeft;
  public static int intakeLeftSolenoid1;
  public static int intakeLeftSolenoid2;
  public static int intakeRight;
  public static int intakeRightSolenoid1;
  public static int intakeRightSolenoid2;
  public static int intakePCM;

  public RobotMap() {
    switch (robot) {
    case ROBOT_2017:
      rightMaster = 14;
      rightSlave = 13;
      rightSlave2 = 12;
      leftMaster = 15;
      leftSlave = 0;
      leftSlave2 = 1;
      maxVelocityLow = 3284; // 525 RPM
      minVelocityLow = 135; // 20 RPM
      maxAcceleration = 300;
      robotLength = 31.5;
      robotWidth = 29.25;
      break;
    case ORIGINAL_ROBOT_2018:
      rightMaster = 2;
      rightSlave = 0;
      leftMaster = 12;
      leftSlave = 13;
      leftDriveGearSolenoid1 = 0;
      leftDriveGearSolenoid2 = 1;
      leftDriveGearPCM = 1;
      rightDriveGearSolenoid1 = 2;
      rightDriveGearSolenoid2 = 3;
      rightDriveGearPCM = 0;
      maxVelocityHigh = 7056; // ~230 in/s
      maxVelocityLow = 3252; // 106 in/s
      minVelocityLow = 100;
      minVelocityHigh = 400;
      robotLength = 32 + 6;
      robotWidth = 27 + 6;
      break;
    case EVERYBOT_2019:
      rightMaster = 0;
      leftMaster = 15;
      rightSlave = 1;
      leftSlave = 14;
      maxVelocityLow = 950; // 950 native units per 100ms
      minVelocityLow = 40; // 40 native units per 100ms
      maxAcceleration = 300;
      break;
    case ROBOT_2019:
    case ROBOT_2019_2:
      rightMaster = 0;
      rightSlave = 0;
      rightSlave2 = 0;
      leftMaster = 0;
      leftSlave = 0;
      leftSlave2 = 0;
      vacuumMotor = 0;
      maxVelocityLow = 0;
      minVelocityLow = 0;
      maxAcceleration = 0;
      intakePCM = 0;
      intakeLeftSolenoid1 = 1;
      intakeLeftSolenoid2 = 2;
      intakeRightSolenoid1 = 3;
      intakeRightSolenoid2 = 4;
      break;
    default:
      break;
    }
  }

  public enum RobotType {
    ROBOT_2019, ROBOT_2019_2, ORIGINAL_ROBOT_2018, EVERYBOT_2019, ROBOT_2017
  }
}
