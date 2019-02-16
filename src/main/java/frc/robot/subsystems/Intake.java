/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.RobotMap.RobotType;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {

  private static final int configTimeout = 0;
  private static final NeutralMode brakeMode = NeutralMode.Brake;

  private double intakeSpeed;
  private double ejectSpeed;
  private boolean intakeSpeedLocked;
  private boolean ejectSpeedLocked;
  private boolean enableCurrentLimit;
  private boolean invertLeft;
  private boolean invertRight;
  private int continuousCurrentLimit;
  private int peakCurrentLimit;
  private int peakCurrentLimitDuration; // in milliseconds

  VictorSPX leftVictor;
  VictorSPX rightVictor;
  DoubleSolenoid retractSolenoidLeft;
  DoubleSolenoid retractSolenoidRight;

  public Intake() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      retractSolenoidLeft = new DoubleSolenoid(RobotMap.intakePCM, RobotMap.intakeLeftSolenoid1,
          RobotMap.intakeLeftSolenoid2);
      retractSolenoidRight = new DoubleSolenoid(RobotMap.intakePCM, RobotMap.intakeRightSolenoid1,
          RobotMap.intakeRightSolenoid2);
      intakeSpeed = 1;
      ejectSpeed = 1;
      intakeSpeedLocked = false;
      ejectSpeedLocked = false;
      invertLeft = true;
      invertRight = false;
      enableCurrentLimit = true;
      continuousCurrentLimit = 0;
      peakCurrentLimit = 0;
      peakCurrentLimitDuration = 0;
    }
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      leftVictor = new VictorSPX(RobotMap.intakeLeft);
      rightVictor = new VictorSPX(RobotMap.intakeRight);
      // Are we using TalonSRX or VictorSPX?

    }
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
