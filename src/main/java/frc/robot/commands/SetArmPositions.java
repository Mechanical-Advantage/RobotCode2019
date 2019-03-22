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
import frc.robot.subsystems.Arm;


public class SetArmPositions extends Command {

  private static ArmPosition targetArmPosition; // to recieve the next-arm-location instructions (which ultimately comes from operator station)
  public static Double targetTelescope; // how far to extend telescope
  public static Double targetElbow; // angle to set elbow to
  public static Double targetWrist; // angle to set wrist to

  // values for live small-movements (target adjustments)
  public static double moveXBy = 0.0; // this value could be sent from the operator station to move the target location forward by x inches (negative for backward)
  public static double moveYBy = 0.0; // this value could be sent from the operator station to move the target location up by y inches
  // should add SET and GET methods to these two variables above so operator can adjust position live

  // declare variables to hold the mapped values, initialized with sample values
  public static boolean targetXFrontValue; // lookup in map if in front of robot (else behind)
  public static double targetXValue; // lookup physical measurement in inches in map, measured from edge of bumper
  public static double targetYValue; // lookup physical measurement in inches in map, measured up from floor
  public static double targetNozzleAngleValue;  // assumes 0 is horizontal, 45 and 90 are angled downward from horizontal 
  public static double targetXFromElbowValue;  // x distance from elbow to wrist
  public static double targetYFromElbowValue;  // y distance from elbow to wrist 
  public static boolean shoulderUp; //  is the sholder raised?
  public static double shoulderAngle; // holds shoulder joint angle of rotation
  public static double bicepX; // x component length of bicep [Arm.java makes this NEGATIVE!]
  public static double bicepY; // y component length of bicep
  public static double nozzleX;  // x component length of vaccum nozzle
  public static double nozzleY;  // y component length of vaccum nozzle
  private static double arctanElbow;

  // Constants in inches
  public static final double nozzleLength = 5.0;  //nozzle is vacuum piece to suck up games pieces, connected to wrist  THIS IS AN ESTIMATE NEEDS TO BE MEASURED!
  public static final double shoulderMountHeight = 18.0; //above ground THIS IS AN ESTIMATE NEEDS TO BE MEASURED!

  public enum ArmPosition {
    ROCKET_HI_PLATE, ROCKET_HI_CARGO, ROCKET_MID_PLATE, ROCKET_MID_CARGO, ROCKET_LO_PLATE, ROCKET_LO_CARGO, CARGOSHIP_PLATE, CARGOSHIP_CARGO, FLOOR_PLATE, FLOOR_CARGO, LOADING_PLATE, LOADING_CARGO;
    
    //The values set below are just estimates- still need to be checked/tested!
    private static final Map<ArmPosition,Boolean> shoulderMap = Map.ofEntries(Map.entry(ROCKET_HI_PLATE, true),  
                                                                      Map.entry(ROCKET_HI_CARGO, true),
                                                                      Map.entry(ROCKET_MID_PLATE, false),
                                                                      Map.entry(ROCKET_MID_CARGO, false),
                                                                      Map.entry(ROCKET_LO_PLATE, false),  
                                                                      Map.entry(ROCKET_LO_CARGO, false),
                                                                      Map.entry(CARGOSHIP_PLATE, true),
                                                                      Map.entry(CARGOSHIP_CARGO, true),
                                                                      Map.entry(FLOOR_PLATE, false),
                                                                      Map.entry(FLOOR_CARGO, false),
                                                                      Map.entry(LOADING_PLATE, false),
                                                                      Map.entry(LOADING_CARGO, false));

    private static final Map<SetArmPositions.ArmPosition,Double> wristMap = Map.ofEntries(Map.entry(ROCKET_HI_PLATE, 90.0), // units are angle degrees.
                                                                      Map.entry(ROCKET_HI_CARGO, 90.0),
                                                                      Map.entry(ROCKET_MID_PLATE, 90.0),
                                                                      Map.entry(ROCKET_MID_CARGO, 90.0),
                                                                      Map.entry(ROCKET_LO_PLATE, 90.0),
                                                                      Map.entry(ROCKET_LO_CARGO, 90.0),
                                                                      Map.entry(CARGOSHIP_PLATE, 45.0),
                                                                      Map.entry(CARGOSHIP_CARGO, 45.0),
                                                                      Map.entry(FLOOR_PLATE, 0.0),
                                                                      Map.entry(FLOOR_CARGO, 0.0),
                                                                      Map.entry(LOADING_PLATE, -60.0),
                                                                      Map.entry(LOADING_CARGO, -60.0));
                                                                      
    private static final Map<ArmPosition,Double> elbowMap = Map.ofEntries(Map.entry(ROCKET_HI_PLATE, 135.0),
                                                                  Map.entry(ROCKET_HI_CARGO, 120.0), 
                                                                  Map.entry(ROCKET_MID_PLATE, 110.0),
                                                                  Map.entry(ROCKET_MID_CARGO, 105.0),
                                                                  Map.entry(ROCKET_LO_PLATE, 170.0),
                                                                  Map.entry(ROCKET_LO_CARGO, 170.0),
                                                                  Map.entry(CARGOSHIP_PLATE, 125.0),
                                                                  Map.entry(CARGOSHIP_CARGO, 125.0),
                                                                  Map.entry(FLOOR_PLATE, -60.0),
                                                                  Map.entry(FLOOR_CARGO, -60.0),
                                                                  Map.entry(LOADING_PLATE, -60.0),
                                                                  Map.entry(LOADING_CARGO, -60.0));
                                                                  
    private static final Map<ArmPosition,Double> telescopeMap = Map.ofEntries(Map.entry(ROCKET_HI_PLATE, 15.0), // probably want longer extension values for higher targets? 
                                                                  Map.entry(ROCKET_HI_CARGO, 20.0),    // lengths are in addition to 30 inches of "forearm"
                                                                  Map.entry(ROCKET_MID_PLATE, 3.0),
                                                                  Map.entry(ROCKET_MID_CARGO, 8.0),
                                                                  Map.entry(ROCKET_LO_PLATE, 0.0),
                                                                  Map.entry(ROCKET_LO_CARGO, 5.0),
                                                                  Map.entry(CARGOSHIP_PLATE, 5.0),
                                                                  Map.entry(CARGOSHIP_CARGO, 0.0),
                                                                  Map.entry(FLOOR_PLATE, 0.0),
                                                                  Map.entry(FLOOR_CARGO, 5.0),
                                                                  Map.entry(LOADING_PLATE, 3.0),
                                                                  Map.entry(LOADING_CARGO, 3.0));


    // BELOW ARE ADDITONS FOR THE MAPS

    private static final Map<ArmPosition,Boolean> targetXFront = Map.ofEntries(Map.entry(ROCKET_HI_PLATE, true),  //if in front of robot (else behind)
                                                                  Map.entry(ROCKET_HI_CARGO, true),
                                                                  Map.entry(ROCKET_MID_PLATE, false),
                                                                  Map.entry(ROCKET_MID_CARGO, false),
                                                                  Map.entry(ROCKET_LO_PLATE, false),  
                                                                  Map.entry(ROCKET_LO_CARGO, false),
                                                                  Map.entry(CARGOSHIP_PLATE, true),
                                                                  Map.entry(CARGOSHIP_CARGO, true),
                                                                  Map.entry(FLOOR_PLATE, false),
                                                                  Map.entry(FLOOR_CARGO, false),
                                                                  Map.entry(LOADING_PLATE, false),
                                                                  Map.entry(LOADING_CARGO, false));

   private static final Map<ArmPosition,Double> targetNozzleAngle = Map.ofEntries(Map.entry(ROCKET_HI_PLATE, 0.0),   //assumes 0 is horizontal, 45 and 90 are angled downward from horizontal
                                                                  Map.entry(ROCKET_HI_CARGO, 0.0), 
                                                                  Map.entry(ROCKET_MID_PLATE, 0.0),
                                                                  Map.entry(ROCKET_MID_CARGO, 0.0),
                                                                  Map.entry(ROCKET_LO_PLATE, 0.0),
                                                                  Map.entry(ROCKET_LO_CARGO, 0.0),
                                                                  Map.entry(CARGOSHIP_PLATE, 0.0),
                                                                  Map.entry(CARGOSHIP_CARGO, 45.0),
                                                                  Map.entry(FLOOR_PLATE, 90.0),
                                                                  Map.entry(FLOOR_CARGO, 45.0),
                                                                  Map.entry(LOADING_PLATE, 0.0),
                                                                  Map.entry(LOADING_CARGO, 0.0));
                                                                  
    private static final Map<ArmPosition,Double> targetX = Map.ofEntries(Map.entry(ROCKET_HI_PLATE, 135.0), //physical measurement to target in inches , measured from edge of bumper
                                                                  Map.entry(ROCKET_HI_CARGO, 5.0), 
                                                                  Map.entry(ROCKET_MID_PLATE, 5.0),
                                                                  Map.entry(ROCKET_MID_CARGO, 5.0),
                                                                  Map.entry(ROCKET_LO_PLATE, 5.0),
                                                                  Map.entry(ROCKET_LO_CARGO, 5.0),
                                                                  Map.entry(CARGOSHIP_PLATE, 5.0),
                                                                  Map.entry(CARGOSHIP_CARGO, 5.0),
                                                                  Map.entry(FLOOR_PLATE, 5.0),
                                                                  Map.entry(FLOOR_CARGO, 5.0),
                                                                  Map.entry(LOADING_PLATE, 5.0),
                                                                  Map.entry(LOADING_CARGO, 5.0));
                                                                     
    private static final Map<ArmPosition,Double> targetY = Map.ofEntries(Map.entry(ROCKET_HI_PLATE, 77.0), //physical measurement to target in inches , measured up from floor
                                                                  Map.entry(ROCKET_HI_CARGO, 87.0), 
                                                                  Map.entry(ROCKET_MID_PLATE, 47.5),
                                                                  Map.entry(ROCKET_MID_CARGO, 57.5),
                                                                  Map.entry(ROCKET_LO_PLATE, 17.5),
                                                                  Map.entry(ROCKET_LO_CARGO, 27.5),
                                                                  Map.entry(CARGOSHIP_PLATE, 19.0),
                                                                  Map.entry(CARGOSHIP_CARGO, 38.0),
                                                                  Map.entry(FLOOR_PLATE, 0.5),
                                                                  Map.entry(FLOOR_CARGO, 5.53),
                                                                  Map.entry(LOADING_PLATE, 5.53),
                                                                  Map.entry(LOADING_CARGO, -45.0));

                                                            
    public static boolean lookUpShoulderPosition(ArmPosition position) {
      return shoulderMap.get(position);
    }

    public static double lookUpTargetNozzleAngle(ArmPosition position) {
      return targetNozzleAngle.get(position);
    }

    public static double lookUpTargetY(ArmPosition position) {
      return targetY.get(position);
    }

    public static double lookUpTargetX(ArmPosition position) {
      return targetX.get(position);
    }

    public static Boolean lookUpTargetXFront(ArmPosition position) {
      return targetXFront.get(position);
    }

    public static double lookUpWristPosition() {
      // return wristMap.get(position);
      return targetWrist;
    }

    public static double lookUpElbowPosition() {
      // return elbowMap.get(position);
      return targetElbow;
    }

    public static double lookUpTelescopePosition() {
      // return telescopeMap.get(position);
      return targetTelescope;
    }

    public static void calculateArmPositions(ArmPosition position) {

      // requires(Robot.arm);

      targetArmPosition = position;
      
      // NEW APPROACH:
        // get the armPosition data here, and use to calculate
        // put calculations here, and set the correct variables
        // just read those variables in the lookUp methods
    
  
      //  **** angles are in degrees
      // positive angles are moving in the counterclockwise direction when viewer is facing the right side of robot; except shoulder angle, which is measured counterclockwise facing the left side of the robot      
      // ** get from map the desired position of the tip of the nozzle and the shoulder position;
  
      // Option 1: read Map values
        shoulderUp = ArmPosition.shoulderMap.get(targetArmPosition);
        targetNozzleAngleValue = ArmPosition.targetNozzleAngle.get(position);
        targetYValue = ArmPosition.targetY.get(position);
        targetXValue = ArmPosition.targetX.get(position);
        targetXFrontValue = ArmPosition.targetXFront.get(position);
    
        // // Option 2: read Map values via methods
        Robot.arm.setShoulderRaised(ArmPosition.lookUpShoulderPosition(targetArmPosition)); // 0 = flat; 1 = up
        shoulderUp = Robot.arm.isShoulderRaised();  // this way plus line above ?
        shoulderUp = ArmPosition.lookUpShoulderPosition(targetArmPosition);  // or this way? 
        targetXFrontValue = ArmPosition.lookUpTargetXFront(targetArmPosition); // lookup in map if in front of robot (else behdind)
        targetXValue = ArmPosition.lookUpTargetX(targetArmPosition) + moveXBy; // lookup physical measurement in inches in map, measured from edge of bumper
        targetYValue = ArmPosition.lookUpTargetY(targetArmPosition) + moveYBy; // lookup physical measurement in inches in map, measured up from floor
        targetNozzleAngleValue = ArmPosition.lookUpTargetNozzleAngle(targetArmPosition) ;  // assumes 0 is horizontal, 45 and 90 are angled downward from horizontal 
        
        // calculate some trig values [**** MAKE REFERENCES FROM Arm.java public, or move them here?]
        shoulderAngle = shoulderUp ? 60 : 0 ; // should pull from Robot.arm.elbowOffsetHigh, but it's private now; assumes this is the angle the bicep makes with the floor when shoulder joint is raised
        bicepX = shoulderUp ? Arm.shoulderDistanceHigh : Arm.shoulderDistanceLow; // NEGATIVE x component of bicep
        bicepY = shoulderUp ? Math.sin(Math.toRadians(Arm.elbowOffsetHigh)) * Arm.bicepLength : Math.sin(Math.toRadians(Arm.elbowOffsetLow)) * Arm.bicepLength; // y component of bicep
        nozzleX = Math.cos(targetNozzleAngleValue) * nozzleLength;
        nozzleY = Math.sin(targetNozzleAngleValue) * nozzleLength;
        
        // compute target distance, as measured from elbow point; positive x is toward front of robot; negative x is toward back of robot 
        if (targetXFrontValue) {
            targetXFromElbowValue = targetXValue + Arm.framePerimeterFrontFromShoulder - bicepX - nozzleX ;  // x distance from elbow to wrist
        } else {
            targetXFromElbowValue = -1 * (targetXValue + Arm.framePerimeterBackFromShoulder + bicepX - nozzleX) ; // negative x for behind robot
        };
        targetYFromElbowValue = targetYValue - shoulderMountHeight - bicepY + nozzleY;  // y distance from elbow to wrist
        
        arctanElbow = Math.toDegrees(Math.atan(targetYFromElbowValue/targetXFromElbowValue));
        
        // compute joint and telescope settings
        targetTelescope = Math.sqrt(targetYFromElbowValue * targetYFromElbowValue + targetXFromElbowValue * targetXFromElbowValue) - Arm.forearmLength; // how far to extend telescope in inches
        
        if (targetXFrontValue) {    // target in front of robot
            targetElbow = arctanElbow + shoulderAngle ; // in degrees, where 0 = pointing horizontal toward front of robot; if this is wrong, change elbowZeroedPosition in Arm.java
            targetWrist = 0 - arctanElbow - targetNozzleAngleValue; 
        } else {    // target behind robot
            targetElbow = 180  + shoulderAngle - arctanElbow ; // in degrees, where 0 = horizontal in front of robot    
            targetWrist = arctanElbow + targetNozzleAngleValue;
        }
        // ** targetWrist assumes 0 degrees for the wrist is when nozzle is pointed in same direction as forearm -- if that's wrong, adjust WristZeroedPosition in Arm.java !!
// **** targetWrist:: Eliott will need to change the type in this method to accept an angle value, or we need to change how the wrist position is stored and read
// **** I don't think we need to call updateWristSetpoint anymore in the function setElbowPosition

 
      // if shoulder was not already set above, use this
      Robot.arm.setShoulderRaised(shoulderUp); // 0 = flat; 1 = up
  
    }

  }


  public SetArmPositions(ArmPosition position) {
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
