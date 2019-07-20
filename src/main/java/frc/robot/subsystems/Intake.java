/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.RobotType;

/**
 * Intake subsystem
 */
public class Intake extends Subsystem {

  private static final boolean intakeReversed = true;
  private static final NeutralMode intakeNeutralMode = NeutralMode.Brake;
  public final boolean autoHold = true;
  private static final Double holdSpeed = -0.1;

  private static final boolean intakeEnableCurrentLimit = true;
  private static final int intakeContinousCurrentLimit = 30; // A
  private static final int intakePeakCurrentLimit = 40; // A
  private static final int intakePeakCurrentLimitDuration = 1000; // ms

  private static final boolean demoIntakeEnableCurrentLimit = true;
  private static final int demoIntakeContinousCurrentLimit = 2; // A
  private static final int demoIntakePeakCurrentLimit = 15; // A
  private static final int demoIntakePeakCurrentLimitDuration = 100; // ms

  private DoubleSolenoid cargoSolenoid;
  private DoubleSolenoid hatchControlSolenoid;
  private DoubleSolenoid hatchReleaseSolenoid;
  private TalonSRX intake;

  private GamePiece gamePiece = GamePiece.HATCH;
  private boolean lastDemoMode = false;

  private boolean available() {
    return RobotMap.robot == RobotType.ROBOT_REBOT;
  }

  public Intake() {
    if (available()) {
      cargoSolenoid = new DoubleSolenoid(RobotMap.rebotPCM, RobotMap.cargoRaiseSolenoid, RobotMap.cargoLowerSolenoid);
      hatchControlSolenoid = new DoubleSolenoid(RobotMap.rebotPCM, RobotMap.hatchOpenSolenoid,
          RobotMap.hatchCloseSolenoid);
      hatchReleaseSolenoid = new DoubleSolenoid(RobotMap.rebotPCM, RobotMap.hatchDeliverSolenoid,
          RobotMap.hatchWithdrawSolenoid);
      intake = new TalonSRX(RobotMap.intakeMotor);
      intake.configFactoryDefault();
      intake.setInverted(intakeReversed);
      intake.setNeutralMode(intakeNeutralMode);
      configCurrentLimits(false);
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  public void periodic() {
    boolean demoMode = Robot.oi.getDemoMode();
    if (lastDemoMode != demoMode) {
      lastDemoMode = demoMode;
      configCurrentLimits(lastDemoMode);
    }
  }

  public void configCurrentLimits(boolean demoLimits) {
    if (demoLimits) {
      intake.configContinuousCurrentLimit(demoIntakeContinousCurrentLimit);
      intake.configPeakCurrentLimit(demoIntakePeakCurrentLimit);
      intake.configPeakCurrentDuration(demoIntakePeakCurrentLimitDuration);
      intake.enableCurrentLimit(demoIntakeEnableCurrentLimit);
    } else {
      intake.configContinuousCurrentLimit(intakeContinousCurrentLimit);
      intake.configPeakCurrentLimit(intakePeakCurrentLimit);
      intake.configPeakCurrentDuration(intakePeakCurrentLimitDuration);
      intake.enableCurrentLimit(intakeEnableCurrentLimit);
    }
  }

  public void run(double power) {
    if (available()) {
      intake.set(ControlMode.PercentOutput, power);
    }
  }

  public void raise() {
    if (available()) {
      cargoSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public void lower() {
    if (available()) {
      cargoSolenoid.set(DoubleSolenoid.Value.kForward);
    }
  }

  public boolean isRaised() {
    if (available()) {
      return cargoSolenoid.get() == Value.kReverse;
    } else {
      return false;
    }
  }

  public void close() {
    if (available()) {
      hatchControlSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public void open() {
    if (available()) {
      hatchControlSolenoid.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void extend() {
    if (available()) {
      hatchReleaseSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public void retract() {
    if (available()) {
      hatchReleaseSolenoid.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void stop() {
    if (available()) {
      intake.neutralOutput();
    }
  }

  public void hold() {
    if (available()) {
      intake.set(ControlMode.PercentOutput, holdSpeed);
    }
  }

  public enum GamePiece {
    HATCH, CARGO;
  }

  public GamePiece getGamepiece() {
    return gamePiece;
  }

  public void setGamepiece(GamePiece piece) {
    gamePiece = piece;
    SmartDashboard.putBoolean("Game Piece", gamePiece == GamePiece.HATCH);
  }
}
