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

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
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

  TalonSRX leftTalon;
  TalonSRX rightTalon;
  DoubleSolenoid retractSolenoidLeft;
  DoubleSolenoid retractSolenoidRight;

  public Intake() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      retractSolenoidLeft = new DoubleSolenoid(RobotMap.intakePCM, RobotMap.intakeLeftSolenoid1,
          RobotMap.intakeLeftSolenoid2);
      retractSolenoidRight = new DoubleSolenoid(RobotMap.intakePCM, RobotMap.intakeRightSolenoid1,
          RobotMap.intakeRightSolenoid2);
      intakeSpeed = 0.5;
      ejectSpeed = 1;
      intakeSpeedLocked = false;
      ejectSpeedLocked = false;
      invertLeft = true;
      invertRight = false;
      enableCurrentLimit = true;
      continuousCurrentLimit = 30;
      peakCurrentLimit = 50;
      peakCurrentLimitDuration = 50;

      leftTalon = new TalonSRX(RobotMap.intakeLeft);
      rightTalon = new TalonSRX(RobotMap.intakeRight);

      leftTalon.enableCurrentLimit(enableCurrentLimit);
      leftTalon.configContinuousCurrentLimit(continuousCurrentLimit, configTimeout);
      leftTalon.configPeakCurrentLimit(peakCurrentLimit, configTimeout);
      leftTalon.configPeakCurrentDuration(peakCurrentLimitDuration, configTimeout);
      leftTalon.setInverted(invertLeft);
      leftTalon.setNeutralMode(brakeMode);
      rightTalon.enableCurrentLimit(enableCurrentLimit);
      rightTalon.configContinuousCurrentLimit(continuousCurrentLimit, configTimeout);
      rightTalon.configPeakCurrentLimit(peakCurrentLimit, configTimeout);
      rightTalon.configPeakCurrentDuration(peakCurrentLimitDuration, configTimeout);
      rightTalon.setInverted(invertRight);
      rightTalon.setNeutralMode(brakeMode);
    }
  }

  public void intake() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      intake(intakeSpeedLocked ? intakeSpeed : Robot.oi.getIntakeLevel());
    }
  }

  public void intake(double speed) {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      leftTalon.set(ControlMode.PercentOutput, speed);
      rightTalon.set(ControlMode.PercentOutput, speed);
    }
  }

  public void eject() {
    eject(ejectSpeedLocked ? ejectSpeed : Robot.oi.getEjectForce());
  }

  public void eject(double speed) {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      // Negative power ejects
      leftTalon.set(ControlMode.PercentOutput, speed * -1);
      rightTalon.set(ControlMode.PercentOutput, speed * -1);
    }
  }

  public void stop() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      leftTalon.neutralOutput();
      rightTalon.neutralOutput();
    }
  }

  public void setRetracted(boolean retracted) {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      retractSolenoidLeft.set(retracted ? Value.kReverse : Value.kForward);
      retractSolenoidRight.set(retracted ? Value.kReverse : Value.kForward);
    }
  }

  public boolean getRetractedLeft() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      // Solenoid is not driven at the beginning of match but is retracted, so check
      // if not extended
      return retractSolenoidLeft.get() != Value.kForward;
    }
    return false;
  }

  public boolean getRetractedRight() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      // Solenoid is not driven at the beginning of match but is retracted, so check
      // if not extended
      return retractSolenoidRight.get() != Value.kForward;
    }
    return false;
  }

  public double getCurrent() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      return (leftTalon.getOutputCurrent() + rightTalon.getOutputCurrent()) / 2;
    } else {
      return 0;
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
