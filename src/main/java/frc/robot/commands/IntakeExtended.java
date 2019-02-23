/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.RobotType;

/**
 * Add your docs here.
 */
public class IntakeExtended extends InstantCommand {
  /**
   * Add your docs here.
   */
  public IntakeExtended() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      if (Robot.intake.getRetractedLeft() && Robot.intake.getRetractedRight()) {
        Robot.intake.intake();
        Robot.intake.setRetracted(false);
      }
  }

}

}
