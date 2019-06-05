/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Intake;

/**
 * Raises or lowers the intake
 */
public class ReBotSetIntakeRaised extends InstantCommand {

  private boolean raised;

  public ReBotSetIntakeRaised(boolean raised) {
    super("ReBotSetIntakeRaised");
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    requires(Robot.intake);
    this.raised = raised;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (raised) {
      Robot.intake.raise();
    } else {
      Robot.intake.lower();
    }
  }

}
