/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.Delay;
import frc.robot.commands.ReBotExtendEjectorPistons;
import frc.robot.commands.ReBotOpenHatchIntake;

public class ReBotEjectHatch extends CommandGroup {

  private final Double ejectorDelay = 0.5; // Time to wait after openning intake before ejecting hatch
  private final Double ejectorDuration = (double) 1; // Time to leave ejector pistons extended

  /**
   * Ejects hatch (opens hatch intake and extends ejector pistons after delay)
   */
  public ReBotEjectHatch() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
    addSequential(new ReBotOpenHatchIntake());
    addSequential(new Delay(ejectorDelay));
    addSequential(new ReBotExtendEjectorPistons(ejectorDuration));
  }
}
