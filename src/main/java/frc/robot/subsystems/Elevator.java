/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.util.SchoolZone;
import frc.robot.util.TunableNumber;

/**
 * This is the elvator subsystem for ROBOT_REBOT
 */
public class Elevator extends Subsystem {
  private static final double elevatorUpperLimit = 0; // Temporary, revise later
  private static final double elevatorLowerLimit = 200; // Temporary, revise later
  private static final double schoolZoneUpperLimit = 150; // Temporary, revise later
  private static final double schoolZoneLowerLimit = 15; // Temporary, revise later

  private static final TunableNumber kPElbow = new TunableNumber("Elevator/p");
  private static final TunableNumber kIElbow = new TunableNumber("Elevator/i");
  private static final TunableNumber kDElbow = new TunableNumber("Elevator/d");

  private TalonSRX rightElevatorMaster;
  private TalonSRX leftElevatorMaster;

  private SchoolZone schoolZone;

  private boolean elevatorEnabled;
  private boolean elevatorLimitsEnabled = true;
  private boolean elevatorOpenLoop = true;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
