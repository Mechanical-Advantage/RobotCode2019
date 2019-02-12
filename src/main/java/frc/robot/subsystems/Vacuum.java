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
import frc.robot.RobotMap;
import frc.robot.RobotMap.RobotType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;



/**
 * Add your docs here.
 */
public class Vacuum extends Subsystem {

  private VictorSPX vacuumMotor;

  private static final boolean reverseVacuumMotor = false;
  private static final boolean vacMotorBrakeMode = false;

  public enum VacSolenoid {
    PUMP_TAIL, PUMP_TANK, TAIL_TANK, PICKUP, ATMOSPHERE;
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
      case ATMOSPHERE:
        channelID = 2;
        break;
      default:
        throw new IndexOutOfBoundsException("Invalid Solenoid ID");
      }
      return (channelID);
    }

    private static int getSubChannelID(VacSolenoid id) {
      int subChannelID;
      switch (id) {
      case PUMP_TAIL:
      case PUMP_TANK:
        subChannelID = 0;
        break;
      case PICKUP:
      case TAIL_TANK:
      case ATMOSPHERE:
        subChannelID = 1;
        break;
      default:
        throw new IndexOutOfBoundsException("Invalid Solenoid ID");
      }
      return (subChannelID);
    }
  }

  private RelayChannel[] relayChannel = new RelayChannel[3];
  AnalogInput pressureSensor;
  private static final int PRESSURE_ANALOG_INPUT = 1;

  public Vacuum() {
    for (int i = 0; i <= 2; i++) {
      relayChannel[i] = new RelayChannel(i);
    }
    pressureSensor = new AnalogInput(PRESSURE_ANALOG_INPUT);
    pressureSensor.setAverageBits(4);

    if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
      vacuumMotor = new VictorSPX(RobotMap.vacuumMotor);
    }

    vacuumMotor.setInverted(reverseVacuumMotor);
    vacuumMotor.setNeutralMode(vacMotorBrakeMode ? NeutralMode.Brake : NeutralMode.Coast);
  }

  public void setVacuumMotor(boolean state) {
    if (state) {
      vacuumMotor.set(ControlMode.PercentOutput, 1);
    } else {
      vacuumMotor.set(ControlMode.PercentOutput, 0);
    }
  }


  public void setSolenoid(VacSolenoid id, boolean isOn) {
    int channelID = VacSolenoid.getChannelID(id);
    int subChannelID = VacSolenoid.getSubChannelID(id);
    relayChannel[channelID].setState(subChannelID, isOn);
  }

  public boolean getSolenoid(VacSolenoid id) {
    int channelID = VacSolenoid.getChannelID(id);
    int subChannelID = VacSolenoid.getSubChannelID(id);
    return relayChannel[channelID].getState(subChannelID);
  }

  public double getPressureSensorVoltage() {
    return pressureSensor.getAverageVoltage();
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
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
