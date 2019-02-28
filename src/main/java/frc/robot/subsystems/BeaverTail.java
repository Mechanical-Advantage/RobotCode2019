/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.RobotType;

/**
 * Beaver tail.
 * Note: Most functionality is part of either Vacuum or DriveTrain (pto methods)
 */
public class BeaverTail extends Subsystem {

  private DoubleSolenoid release;

  public BeaverTail() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      release = new DoubleSolenoid(RobotMap.tailReleasePCM, 
        RobotMap.tailReleaseSolenoid1, RobotMap.tailReleaseSolenoid2);
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void lockTail() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      release.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void releaseTail() {
    if (!Robot.oi.isTailLocked() && RobotMap.robot == RobotType.ROBOT_2019 || 
    RobotMap.robot == RobotType.ROBOT_2019_2) {
      release.set(DoubleSolenoid.Value.kReverse);
    }
  }

  /**
   * Get whether the tail is locked
   * 
   * @return true if tail locked else false
   */
  public boolean getTailLocked() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      return release.get() != DoubleSolenoid.Value.kReverse; // Makes engaged the default state
    } else {
      return true;
    }
  }
}
