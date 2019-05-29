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

/**
 * Intake subsystem
 */
public class Intake extends Subsystem {

  private static final boolean intakeReversed = true;
  private static final NeutralMode intakeNeutralMode = NeutralMode.Coast;

  private static final boolean intakeEnableCurrentLimit = false;
  private static final int intakeContinousCurrentLimit = 0;
  private static final int intakePeakCurrentLimit = 0;
  private static final int intakePeakCurrentLimitDuration = 0; // ms

  private TalonSRX intake;

  private boolean available() {
    return RobotMap.robot == RobotType.ROBOT_2017;
  }

  public Intake() {
    if (available()) {
      intake = new TalonSRX(RobotMap.intakeMotor);

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

  public void run(double power) {
    if (available()) {
      intake.set(ControlMode.PercentOutput, power);
    }
  }

  public void stop() {
    if (available()) {
      intake.neutralOutput();
    }
  }
}
