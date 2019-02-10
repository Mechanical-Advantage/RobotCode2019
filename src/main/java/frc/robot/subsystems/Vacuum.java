/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Vacuum extends Subsystem {
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
      case ATMOSPHERE:
        subChannelID = 0;
        break;
      case PICKUP:
      case TAIL_TANK:
        subChannelID = 1;
        break;
      default:
        throw new IndexOutOfBoundsException("Invalid Solenoid ID");
      }
      return (subChannelID);
    }
  }

  private RelayChannel[] relayChannel = new RelayChannel[3];

  public Vacuum() {
    for (int i = 0; i <= 2; i++) {
      relayChannel[i] = new RelayChannel(i);
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

  private static class RelayChannel {
    private Relay relay;
    private boolean[] state;

    public RelayChannel(int channel) {
      relay = new Relay(channel, Relay.Direction.kBoth);
      state = new boolean[] { false, false };
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
