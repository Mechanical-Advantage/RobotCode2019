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

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.OI.OILED;
import frc.robot.RobotMap.RobotType;

/**
 * The simpler scoring mechanism added before GSD
 */
public class SimpleScorer extends Subsystem {

  private static final boolean intakeReversed = false;
  private static final NeutralMode intakeNeutralMode = NeutralMode.Coast;

  private static final boolean intakeEnableCurrentLimit = false;
  private static final int intakeContinousCurrentLimit = 0;
  private static final int intakePeakCurrentLimit = 0;
  private static final int intakePeakCurrentLimitDuration = 0; // ms

  private DoubleSolenoid extendSolenoid;
  private TalonSRX intake;

  public SimpleScorer() {
    if (available()) {
      extendSolenoid = new DoubleSolenoid(RobotMap.simpleScoringPCM, 
        RobotMap.simpleScoringSolenoid1, RobotMap.simpleScoringSolenoid2);
        intake = new TalonSRX(RobotMap.armLightIntake);

        intake.configFactoryDefault();
        intake.setInverted(intakeReversed);
        intake.setNeutralMode(intakeNeutralMode);
  
        intake.configContinuousCurrentLimit(intakeContinousCurrentLimit);
        intake.configPeakCurrentLimit(intakePeakCurrentLimit);
        intake.configPeakCurrentDuration(intakePeakCurrentLimitDuration);
        intake.enableCurrentLimit(intakeEnableCurrentLimit);  
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  /**
   * Determine whether the subsystem is available on the current robot.
   * 
   * @return Whether the subsystem is available
   */
  private boolean available() {
    return RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2;
  }

  public void extend() {
    if (available()) {
      extendSolenoid.set(DoubleSolenoid.Value.kForward);
      Robot.oi.updateLED(OILED.INTAKE_RETRACT, true);
      Robot.oi.updateLED(OILED.INTAKE_ON_OFF, false);
    }
  }

  public void retract() {
    if (available()) {
      extendSolenoid.set(DoubleSolenoid.Value.kReverse);
      Robot.oi.updateLED(OILED.INTAKE_RETRACT, false);
      Robot.oi.updateLED(OILED.INTAKE_ON_OFF, true);
    }
  }

  public void runIntake(double power) {
    if (available()) {
      intake.set(ControlMode.PercentOutput, power);
    }
  }

  public void stopIntake() {
    if (available()) {
      intake.neutralOutput();
    }
  }
}
