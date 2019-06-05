/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Intake.GamePiece;

/**
 * Sets the gamepiece value in Robot, which determines the height of elevator
 * set points
 */
public class ReBotSetGamepiece extends InstantCommand {

  private GamePiece gamePiece;

  public ReBotSetGamepiece(GamePiece gamePiece) {
    super("ReBotSetGamepiece");
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.gamePiece = gamePiece;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.intake.setGamepiece(gamePiece);
  }

}
