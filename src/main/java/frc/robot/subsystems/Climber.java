/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.RobotMap.RobotType;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
/**
 * Add your docs here.
 */
public class Climber extends Subsystem {

  
  private static final boolean climberMasterReversed = true;
  private static final boolean climberSlaveReversed = true;
  private static final NeutralMode climberNeutralMode = NeutralMode.Coast;

  
  private static final boolean climberEnableCurrentLimit = false;
  private static final int climberContinousCurrentLimit = 0;
  private static final int climberPeakCurrentLimit = 0;
  private static final int climberPeakCurrentLimitDuration = 0; // ms no clue what "Ms" meant
 
 
  private TalonSRX climberMaster;
  private VictorSPX climberSlave;



  public Climber() {
    if (RobotMap.robot == RobotType.ROBOT_2017) {
     climberMaster = new TalonSRX(RobotMap.climberMaster);
     climberSlave = new VictorSPX(RobotMap.climberSlave);

     climberMaster.configFactoryDefault();
     climberMaster.setInverted (climberMasterReversed);
     climberMaster.setNeutralMode (climberNeutralMode);


     climberMaster.configContinuousCurrentLimit (climberContinousCurrentLimit);
     climberMaster.configPeakCurrentLimit (climberPeakCurrentLimit);
     climberMaster.configPeakCurrentDuration (climberPeakCurrentLimitDuration);
     climberMaster.enableCurrentLimit (climberEnableCurrentLimit);
   
     climberSlave.configFactoryDefault();
     climberSlave.setInverted (climberSlaveReversed);
     climberSlave.setNeutralMode (climberNeutralMode);
   
     climberSlave.follow(climberMaster);



    }
  }


  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
