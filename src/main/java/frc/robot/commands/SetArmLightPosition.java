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
import frc.robot.subsystems.ArmLight;

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
    Robot.armLight.enableElbow();
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
    FLOOR_CARGO, LOADING_CARGO, LOADING_PLATE, HOME, UP, CAMERA;

    private static final Map<ArmLightPosition,Boolean> shoulderMap = Map.ofEntries(Map.entry(ROCKET_MID_PLATE, false),
                                                                                   Map.entry(ROCKET_MID_CARGO, false),
                                                                                   Map.entry(ROCKET_LO_PLATE, false),  
                                                                                   Map.entry(ROCKET_LO_CARGO, false),
                                                                                   Map.entry(CARGOSHIP_PLATE, false),
                                                                                   Map.entry(CARGOSHIP_CARGO, false),
                                                                                   Map.entry(FLOOR_CARGO, false),
                                                                                   Map.entry(LOADING_CARGO, false),
                                                                                   Map.entry(LOADING_PLATE, false),
                                                                                   Map.entry(HOME, false),
                                                                                   Map.entry(UP, false),
                                                                                   Map.entry(CAMERA, false));

    private static final Map<ArmLightPosition,Double> elbowMap = Map.ofEntries(Map.entry(ROCKET_MID_PLATE, 110.0),
                                                                               Map.entry(ROCKET_MID_CARGO, 105.0),
                                                                               Map.entry(ROCKET_LO_PLATE, 155.0),
                                                                               Map.entry(ROCKET_LO_CARGO, 123.0), // good
                                                                               Map.entry(CARGOSHIP_PLATE, 155.0),
                                                                               Map.entry(CARGOSHIP_CARGO, 80.0),
                                                                               Map.entry(FLOOR_CARGO, 203.0), // good
                                                                               Map.entry(LOADING_CARGO, 90.0), // good
                                                                               Map.entry(LOADING_PLATE, 163.0),
                                                                               Map.entry(HOME, ArmLight.elbowStartingPosition),
                                                                               Map.entry(UP, 49.0),
                                                                               Map.entry(CAMERA, 35.0)); // good

    public Boolean isShoulderRaised() {
      return shoulderMap.get(this);
    }

    public Double getElbowPosition() {
      return elbowMap.get(this);
    }
  }
}
