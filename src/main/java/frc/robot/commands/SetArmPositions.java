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
import frc.robot.commands.SetArmPositions.ArmPosition;

public class SetArmPositions extends Command {

  public enum ArmPosition {
    ROCKET_HI_PLATE, ROCKET_HI_CARGO, ROCKET_MID_PLATE, ROCKET_MID_CARGO, ROCKET_LO_PLATE, ROCKET_LO_CARGO, CARGOSHIP_PLATE, CARGOSHIP_CARGO, FLOOR_PLATE, FLOOR_CARGO, LOADING_PICKUP;
    

    private static final Map<ArmPosition,Boolean> shoulderMap = Map.of(ROCKET_HI_PLATE, false, 
                                                                      ROCKET_HI_CARGO, false,
                                                                      ROCKET_MID_PLATE, false,
                                                                      ROCKET_MID_CARGO, false,
                                                                      ROCKET_LO_PLATE, false,
                                                                      ROCKET_LO_CARGO, false,
                                                                      CARGOSHIP_PLATE, false,
                                                                      CARGOSHIP_CARGO, false,
                                                                      FLOOR_PLATE, false,
                                                                      FLOOR_CARGO, false,
                                                                      LOADING_PICKUP, false);

    private static final Map<ArmPosition,Double> wristMap = Map.of(ROCKET_HI_PLATE, 0, 
                                                                      ROCKET_HI_CARGO, 0,
                                                                      ROCKET_MID_PLATE, 0,
                                                                      ROCKET_MID_CARGO, 0,
                                                                      ROCKET_LO_PLATE, 0,
                                                                      ROCKET_LO_CARGO, 0,
                                                                      CARGOSHIP_PLATE, 0,
                                                                      CARGOSHIP_CARGO, 0,
                                                                      FLOOR_PLATE, 0,
                                                                      FLOOR_CARGO, 0,
                                                                      LOADING_PICKUP, 0);

    private static final Map<ArmPosition,Double> elbowMap = Map.of(ROCKET_HI_PLATE, 0, 
                                                                  ROCKET_HI_CARGO, 0,
                                                                  ROCKET_MID_PLATE, 0,
                                                                  ROCKET_MID_CARGO, 0,
                                                                  ROCKET_LO_PLATE, 0,
                                                                  ROCKET_LO_CARGO, 0,
                                                                  CARGOSHIP_PLATE, 0,
                                                                  CARGOSHIP_CARGO, 0,
                                                                  FLOOR_PLATE, 0,
                                                                  FLOOR_CARGO, 0,
                                                                  LOADING_PICKUP, 0);
    private static final Map<ArmPosition,Double> telescopeMap = Map.of(ROCKET_HI_PLATE, 0, 
                                                                  ROCKET_HI_CARGO, 0,
                                                                  ROCKET_MID_PLATE, 0,
                                                                  ROCKET_MID_CARGO, 0,
                                                                  ROCKET_LO_PLATE, 0,
                                                                  ROCKET_LO_CARGO, 0,
                                                                  CARGOSHIP_PLATE, 0,
                                                                  CARGOSHIP_CARGO, 0,
                                                                  FLOOR_PLATE, 0,
                                                                  FLOOR_CARGO, 0,
                                                                  LOADING_PICKUP, 0);
    public lookUpPosition () {  // still need to add more here
      // ArmPosition.shoulderMap = Robot.Arm.;

    }                                                                  
  
  private int armPositions;

  public SetArmPositions() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
   
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  
    switch() {      // call to arm subsystem to set
      case ELBOW:
        return Robot.Subsystems.Arm();
      case TELESCOPE:
        return 0;

    }
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
