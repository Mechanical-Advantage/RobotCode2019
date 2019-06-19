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
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.RobotMap.RobotType;

import frc.robot.commands.ReBotRunClimberWithJoystick;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {

  private static final boolean masterReversed = false;
  private static final boolean slaveReversed = true;
  private static final NeutralMode neutralMode = NeutralMode.Coast;

  private static final boolean enableCurrentLimit = false;
  private static final int continousCurrentLimit = 0;
  private static final int peakCurrentLimit = 0;
  private static final int peakCurrentLimitDuration = 0; // ms

  private TalonSRX climberMaster;
  private VictorSPX climberSlave;

  public Climber() {
    if (RobotMap.robot == RobotType.ROBOT_REBOT) {
      climberMaster = new TalonSRX(RobotMap.climberMaster);
      climberSlave = new VictorSPX(RobotMap.climberSlave);

      climberMaster.configFactoryDefault();
      climberMaster.setInverted(masterReversed);
      climberMaster.setNeutralMode(neutralMode);

      climberMaster.configContinuousCurrentLimit(continousCurrentLimit);
      climberMaster.configPeakCurrentLimit(peakCurrentLimit);
      climberMaster.configPeakCurrentDuration(peakCurrentLimitDuration);
      climberMaster.enableCurrentLimit(enableCurrentLimit);

      climberSlave.configFactoryDefault();
      climberSlave.setInverted(slaveReversed);
      climberSlave.setNeutralMode(neutralMode);

      climberSlave.follow(climberMaster);
    }
  }

  private boolean available() {
    return RobotMap.robot == RobotType.ROBOT_REBOT;
  }

  public void run(double power) {
    if (available()) {
      climberMaster.set(ControlMode.PercentOutput, power);
    }
  }

  public void stop() {
    if (available()) {
      climberMaster.neutralOutput();
    }
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ReBotRunClimberWithJoystick());
  }
}
