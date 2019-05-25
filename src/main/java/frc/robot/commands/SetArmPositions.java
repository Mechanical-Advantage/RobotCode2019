/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj.command.Command;

public class SetArmPositions extends Command {

  public enum ArmPosition {
    ROCKET_HI_PLATE, ROCKET_HI_CARGO, ROCKET_MID_PLATE, ROCKET_MID_CARGO, ROCKET_LO_PLATE, ROCKET_LO_CARGO,
    CARGOSHIP_PLATE, CARGOSHIP_CARGO, FLOOR_PLATE, FLOOR_CARGO, LOADING_PICKUP_BACKWARDS, HOME;

    // The values set below are just estimates- still need to be checked/tested!

    private static final Map<ArmPosition, Boolean> shoulderMap = Map.ofEntries(Map.entry(ROCKET_HI_PLATE, true),
        Map.entry(ROCKET_HI_CARGO, true), Map.entry(ROCKET_MID_PLATE, false), Map.entry(ROCKET_MID_CARGO, false),
        Map.entry(ROCKET_LO_PLATE, false), Map.entry(ROCKET_LO_CARGO, false), Map.entry(CARGOSHIP_PLATE, true),
        Map.entry(CARGOSHIP_CARGO, true), Map.entry(FLOOR_PLATE, false), Map.entry(FLOOR_CARGO, false),
        Map.entry(LOADING_PICKUP_BACKWARDS, false));

    private static final Map<SetArmPositions.ArmPosition, Double> wristMap = Map.ofEntries(
        Map.entry(ROCKET_HI_PLATE, 90.0), // units are angle degrees.
        Map.entry(ROCKET_HI_CARGO, 90.0), Map.entry(ROCKET_MID_PLATE, 90.0), Map.entry(ROCKET_MID_CARGO, 90.0),
        Map.entry(ROCKET_LO_PLATE, 90.0), Map.entry(ROCKET_LO_CARGO, 90.0), Map.entry(CARGOSHIP_PLATE, 45.0),
        Map.entry(CARGOSHIP_CARGO, 45.0), Map.entry(FLOOR_PLATE, 0.0), Map.entry(FLOOR_CARGO, 0.0),
        Map.entry(LOADING_PICKUP_BACKWARDS, 45.0));
    // private double getAngle() {
    // switch (this) {
    // case UPRIGHT:
    // return 0;
    // case FLAT:
    // return 90;
    // case CARGO_PICKUP:
    // return 45;
    // }
    // return 0;
    // }

    private static final Map<ArmPosition, Double> elbowMap = Map.ofEntries(Map.entry(ROCKET_HI_PLATE, 135.0),
        Map.entry(ROCKET_HI_CARGO, 120.0), Map.entry(ROCKET_MID_PLATE, 110.0), Map.entry(ROCKET_MID_CARGO, 105.0),
        Map.entry(ROCKET_LO_PLATE, 170.0), Map.entry(ROCKET_LO_CARGO, 170.0), Map.entry(CARGOSHIP_PLATE, 125.0),
        Map.entry(CARGOSHIP_CARGO, 125.0), Map.entry(FLOOR_PLATE, -60.0), Map.entry(FLOOR_CARGO, -60.0),
        Map.entry(LOADING_PICKUP_BACKWARDS, -45.0));

    // probably want longer extension values for higher targets
    private static final Map<ArmPosition, Double> telescopeMap = Map.ofEntries(Map.entry(ROCKET_HI_PLATE, 15.0),
        Map.entry(ROCKET_HI_CARGO, 20.0), // lengths are in addition to 30 inches of "forearm"
        Map.entry(ROCKET_MID_PLATE, 3.0), Map.entry(ROCKET_MID_CARGO, 8.0), Map.entry(ROCKET_LO_PLATE, 0.0),
        Map.entry(ROCKET_LO_CARGO, 5.0), Map.entry(CARGOSHIP_PLATE, 5.0), Map.entry(CARGOSHIP_CARGO, 0.0),
        Map.entry(FLOOR_PLATE, 0.0), Map.entry(FLOOR_CARGO, 5.0), Map.entry(LOADING_PICKUP_BACKWARDS, 3.0));

    public boolean lookUpShoulderPosition(ArmPosition position) {
      return shoulderMap.get(position);
    }

    public double lookUpWristPosition(ArmPosition position) {
      return wristMap.get(position);
    }

    public double lookUpElbowPosition(ArmPosition position) {
      return elbowMap.get(position);
    }

    public double lookUpTelescopePosition(ArmPosition position) {
      return telescopeMap.get(position);
    }
  }

  public SetArmPositions(ArmPosition position) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
