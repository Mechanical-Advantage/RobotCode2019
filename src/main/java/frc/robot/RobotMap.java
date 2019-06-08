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
  public static int leftDriveGearSolenoid1;
  public static int leftDriveGearSolenoid2;
  public static int leftDriveGearPCM;
  public static int rightDriveGearSolenoid1;
  public static int rightDriveGearSolenoid2;
  public static int rightDriveGearPCM;
  public static int ptoSolenoid1;
  public static int ptoSolenoid2;
  public static int ptoSolenoidPCM;
  public static double robotWidth;
  public static double robotLength;
  public static final boolean tuningMode = false;
  public static final RobotType robot = RobotType.ROBOT_REBOT;
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
  public static int vacuumMotor;
  public static boolean vacuumMotorTalon;
  public static int vacuumPressureSensor;
  public static int greenLEDRing; // Currently requires an entire relay channel
  public static int tailReleaseSolenoid1;
  public static int tailReleaseSolenoid2;
  public static int tailReleasePCM;
  public static int simpleScoringSolenoid1;
  public static int simpleScoringSolenoid2;
  public static int simpleScoringPCM;
  public static int simpleScoringIntakeMotor;
  public static int intakeMotor;
  public static int level2FrontPCM;
  public static int level2FrontSolenoid1;
  public static int level2FrontSolenoid2;
  public static int level2RearPCM;
  public static int level2RearSolenoid1;
  public static int level2RearSolenoid2;
  public static int rebotPCM;
  public static int cargoRaiseSolenoid;
  public static int cargoLowerSolenoid;
  public static int hatchOpenSolenoid;
  public static int hatchCloseSolenoid;
  public static int hatchDeliverSolenoid;
  public static int hatchWithdrawSolenoid;
  public static int climberMaster;
  public static int climberSlave;
  public static int elevatorMaster;
  public static int elevatorSlave;

  public RobotMap() {
    switch (robot) {
    case ROBOT_REBOT:
      rightMaster = 1;
      rightSlave = 2;
      rightSlave2 = 3;
      leftMaster = 12;
      leftSlave = 13;
      leftSlave2 = 14;
      maxVelocityLow = 3284; // 525 RPM
      minVelocityLow = 135; // 20 RPM
      maxAcceleration = 300;
      robotLength = 31.5;
      robotWidth = 29.25;
      intakeMotor = 6;
      cargoRaiseSolenoid = 7;
      cargoLowerSolenoid = 6;
      hatchCloseSolenoid = 4;
      hatchOpenSolenoid = 5;
      hatchDeliverSolenoid = 0;
      hatchWithdrawSolenoid = 1;
      rebotPCM = 0;
      climberMaster = 5;
      climberSlave = 10;
      elevatorMaster = 4;
      elevatorSlave = 11;
      vacuumMotor = 7;
      vacuumMotorTalon = true;
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
      rightMaster = 3;
      rightSlave = 2;
      rightSlave2 = 1;
      leftMaster = 12;
      leftSlave = 14;
      leftSlave2 = 13;
      maxVelocityLow = 4800; // 5710 on blocks
      minVelocityLow = 210;
      maxAcceleration = 0;
      armElbowLeft = 15;
      armElbowRight = 0;
      armTelescope = 5;
      armWrist = 11;
      vacuumMotor = 4;
      vacuumMotorTalon = false;
      vacuumPressureSensor = 1;
      armShoulder1Extend = 0; // Left, disabled, using disconnected channels, was PCM 1 extend 4 retract 6
      armShoulder1Retract = 1;
      armShoulder1PCM = 0;
      armShoulder2Extend = 2; // Right, disabled, was PCM 1 extend 5 retract 7
      armShoulder2Retract = 3;
      armShoulder2PCM = 0;
      greenLEDRing = 2;
      ptoSolenoid1 = 3;
      ptoSolenoid2 = 2;
      ptoSolenoidPCM = 1;
      tailReleaseSolenoid1 = 0;
      tailReleaseSolenoid2 = 1;
      tailReleasePCM = 1;
      simpleScoringPCM = 0;
      simpleScoringSolenoid1 = 7;
      simpleScoringSolenoid2 = 5;
      simpleScoringIntakeMotor = 10;
      intakeMotor = 11;
      level2FrontPCM = 1;
      level2FrontSolenoid1 = 5;
      level2FrontSolenoid2 = 7;
      level2RearPCM = 1;
      level2RearSolenoid1 = 4;
      level2RearSolenoid2 = 6;
      climberMaster = 15;
      climberSlave = 14;
      break;
    default:
      break;
    }
  }

  public enum RobotType {
    ROBOT_2019, ROBOT_2019_2, ORIGINAL_ROBOT_2018, EVERYBOT_2019, ROBOT_REBOT
  }
}
