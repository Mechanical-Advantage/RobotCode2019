/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.RobotMap.RobotType;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * Robot Vacuum System
 */
public class Vacuum extends Subsystem {

  private static final boolean reverseVacuumMotor = true;
  private static final boolean vacMotorBrakeMode = false;
  private static final double vacMotorMaxVoltage = 12.0; // Max pump operating voltage
  private static final boolean vacMotorEnableVoltageCompensation = true;
  private static final double vacMotorOutputLOW = .4; // % output to "hold"
  private static final double vacMotorOutputHIGH = .6; // % output to draw initial vacuum

  public enum VacuumLevel {
    OFF, LOW, HIGH;
  }

  private RelayChannel[] relayChannel = new RelayChannel[2];
  private AnalogInput pressureSensor;
  private VictorSPX vacuumMotor;
  // private PowerDistributionPanel pdp = new PowerDistributionPanel();

  public Vacuum() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      for (int i = 0; i <= 1; i++) {
        relayChannel[i] = new RelayChannel(i);
      }
      pressureSensor = new AnalogInput(RobotMap.vacuumPressureSensor);
      pressureSensor.setAverageBits(4);

      vacuumMotor = new VictorSPX(RobotMap.vacuumMotor);

      vacuumMotor.setInverted(reverseVacuumMotor);
      vacuumMotor.setNeutralMode(vacMotorBrakeMode ? NeutralMode.Brake : NeutralMode.Coast);
      vacuumMotor.configVoltageCompSaturation(vacMotorMaxVoltage);
      vacuumMotor.enableVoltageCompensation(vacMotorEnableVoltageCompensation);
    }
  }

  @Override
  public void periodic() {
    if (RobotMap.tuningMode) {
      SmartDashboard.putNumber("Vacuum Voltage", getPressureSensorVoltage());
    }
  }

  public void setVacuumMotor(VacuumLevel state) {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      switch (state) {
      case OFF:
        vacuumMotor.set(ControlMode.PercentOutput, 0);
        break;
      case LOW:
        vacuumMotor.set(ControlMode.PercentOutput, vacMotorOutputLOW);
        break;
      case HIGH:
        vacuumMotor.set(ControlMode.PercentOutput, vacMotorOutputHIGH);
        break;
      }
    }
  }

  public void setSolenoid(VacSolenoid id, boolean isOn) {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      int channelID = VacSolenoid.getChannelID(id);
      int subChannelID = VacSolenoid.getSubChannelID(id);
      relayChannel[channelID].setState(subChannelID, isOn);
    }
  }

  public boolean getSolenoid(VacSolenoid id) {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      int channelID = VacSolenoid.getChannelID(id);
      int subChannelID = VacSolenoid.getSubChannelID(id);
      return relayChannel[channelID].getState(subChannelID);
    }
    return false;
  }

  public double getPressureSensorVoltage() {
    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      return pressureSensor.getAverageVoltage();
    }
    return 0;
  }

  private static class RelayChannel {
    private Relay relay;
    private boolean[] state;

    public RelayChannel(int channel) {
      relay = new Relay(channel, Relay.Direction.kBoth);
      state = new boolean[] { false, false };
      applyState();
    }

    public boolean getState(int subChannel) {
      return state[subChannel];
    }

    public void setState(int subChannel, boolean newState) {
      state[subChannel] = newState;
      applyState();
    }

    private void applyState() {
      Relay.Value output;

      // The relay boards used on 2019 robot are active low inputs.
      // This means a Rio output that is set to ON turns the relay OFF,
      // and a Rio output that is set to OFF turns the relay ON.
      // The Rio sets all outputs to OFF in its power up (reset) state, which
      // means that all relays will be ON at power up/prior to robot enable.
      // As a consequence, the actuators (solenoids) - which are assumed to
      // be normally-closed - should be wired to the "Normally Closed" relay
      // terminals, which will actually be open in the reset state. When
      // the Rio output is set ON, the relay turns OFF, the normally
      // closed contacts are again closed, the solenoid is powered and opens.
      if (state[0] == false) {
        if (state[1] == false) {
          output = Value.kOff;
        } else {
          output = Value.kReverse;
        }
      } else {
        if (state[1] == false) {
          output = Value.kForward;
        } else {
          output = Value.kOn;
        }
      }
      relay.set(output);
    }
  }

  public enum VacSolenoid {
    PUMP_TAIL, PUMP_TANK, TAIL_TANK, PICKUP;
    private static int getChannelID(VacSolenoid id) {
      int channelID;
      switch (id) {
      case PUMP_TAIL:
      case PICKUP:
        channelID = 0;
        break;
      case PUMP_TANK:
      case TAIL_TANK:
        channelID = 1;
        break;
      default:
        throw new IndexOutOfBoundsException("Invalid Solenoid ID");
      }
      return (channelID);
    }

    private static int getSubChannelID(VacSolenoid id) {
      int subChannelID;
      switch (id) {
      case PICKUP:
      case PUMP_TANK:
        subChannelID = 0;
        break;
      case PUMP_TAIL:
      case TAIL_TANK:
        subChannelID = 1;
        break;
      default:
        throw new IndexOutOfBoundsException("Invalid Solenoid ID");
      }
      return (subChannelID);
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
