/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Closes the hatch intake
 */
public class ReBotCloseHatchIntake extends InstantCommand {
  public ReBotCloseHatchIntake() {
    super("ReBotCloseHatchIntake");
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    // requires(Robot.reBotIntake);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    // Robot.reBotIntake.closeHatchIntake();
    Robot.gamePiece = Robot.GamePiece.HATCH;
  }

}
