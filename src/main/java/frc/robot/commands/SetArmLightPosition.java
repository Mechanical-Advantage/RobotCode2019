/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class SetArmLightPosition extends Command {

  private static final double endTolerance = 1; // deg of error to allow when ending command
  
  private ArmLightPosition position;
  
  public SetArmLightPosition(ArmLightPosition position) {
    super();
    requires(Robot.armLight);
    this.position = position;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.armLight.setShoulderRaised(position.isShoulderRaised());
    Robot.armLight.setElbowPosition(position.getElbowPosition());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Math.abs(Robot.armLight.getElbowPosition() - Robot.armLight.getElbowTargetPosition()) 
    <= endTolerance;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    // Allow the subsystem to continue controlling to the position
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.armLight.disableElbow();
  }

  // This is based on the ArmPosition enum from SetArmPositions
  public enum ArmLightPosition {
    ROCKET_MID_PLATE, ROCKET_MID_CARGO, ROCKET_LO_PLATE, ROCKET_LO_CARGO, CARGOSHIP_PLATE, CARGOSHIP_CARGO,
    FLOOR_CARGO, LOADING_PICKUP_BACKWARDS, LOADING_PICKUP;

    private static final Map<ArmLightPosition,Boolean> shoulderMap = Map.ofEntries(Map.entry(ROCKET_MID_PLATE, false),
                                                                                   Map.entry(ROCKET_MID_CARGO, false),
                                                                                   Map.entry(ROCKET_LO_PLATE, false),  
                                                                                   Map.entry(ROCKET_LO_CARGO, false),
                                                                                   Map.entry(CARGOSHIP_PLATE, true),
                                                                                   Map.entry(CARGOSHIP_CARGO, true),
                                                                                   Map.entry(FLOOR_CARGO, false),
                                                                                   Map.entry(LOADING_PICKUP_BACKWARDS, false),
                                                                                   Map.entry(LOADING_PICKUP, false));

    private static final Map<ArmLightPosition,Double> elbowMap = Map.ofEntries(Map.entry(ROCKET_MID_PLATE, 110.0),
                                                                               Map.entry(ROCKET_MID_CARGO, 105.0),
                                                                               Map.entry(ROCKET_LO_PLATE, 170.0),
                                                                               Map.entry(ROCKET_LO_CARGO, 170.0),
                                                                               Map.entry(CARGOSHIP_PLATE, 125.0),
                                                                               Map.entry(CARGOSHIP_CARGO, 125.0),
                                                                               Map.entry(FLOOR_CARGO, -60.0),
                                                                               Map.entry(LOADING_PICKUP_BACKWARDS, -45.0));

    public Boolean isShoulderRaised() {
      return shoulderMap.get(this);
    }

    public Double getElbowPosition() {
      return elbowMap.get(this);
    }
  }
}
