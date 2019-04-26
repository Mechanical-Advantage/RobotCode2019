/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.RobotMap.RobotType;

/**
 * Controls the pistons used 
 */
public class Level2Climber extends Subsystem {

  private DoubleSolenoid front;
  private DoubleSolenoid rear;

  public Level2Climber() {
    if (isAvailable()) {
      front = new DoubleSolenoid(RobotMap.level2FrontPCM, RobotMap.level2FrontSolenoid1, RobotMap.level2FrontSolenoid2);
      rear = new DoubleSolenoid(RobotMap.level2RearPCM, RobotMap.level2RearSolenoid1, RobotMap.level2RearSolenoid2);
    }
  }

  private boolean isAvailable() {
    return RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void setFrontExtended(boolean extend) {
    if (isAvailable()) {
      front.set(extend ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }
  }
  public void setRearExtended(boolean extend) {
    if (isAvailable()) {
      rear.set(extend ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }
  }

  public boolean isFrontExtended() {
    if (isAvailable()) {
      return front.get() == DoubleSolenoid.Value.kForward;
    } else {
      return false;
    }
  }
  public boolean isRearExtended() {
    if (isAvailable()) {
      return rear.get() == DoubleSolenoid.Value.kForward;
    } else {
      return false;
    }
  }
}
