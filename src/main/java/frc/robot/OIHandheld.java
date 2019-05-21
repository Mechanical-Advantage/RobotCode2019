package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;

public class OIHandheld implements OI {
    private boolean joysticksReversed = false;

    // map driver controller to ID 0 and operator controller to ID 1 in driver station
    private XboxController driverController = new XboxController(0);
    private XboxController operatorController = new XboxController(1);

    public OIHandheld() {
        resetRumble();
    }
    
    public double getLeftAxis() {
        if (joysticksReversed) {
			return driverController.getY(Hand.kRight) * -1;
		} else {
			return driverController.getY(Hand.kLeft);
		}
    }

    public double getRightAxis() {
        if (joysticksReversed) {
			return driverController.getY(Hand.kLeft) * -1;
		} else {
			return driverController.getY(Hand.kRight);
		}
    }

    public double getSingleDriveAxis() {
        if (joysticksReversed) {
			return driverController.getY(Hand.kLeft) * -1;
		} else {
			return driverController.getY(Hand.kLeft);
		}
    }

    public double getLeftHorizDriveAxis() {
        return driverController.getX(Hand.kLeft);
    }

    public double getRightHorizDriveAxis() {
        return driverController.getX(Hand.kRight);
    }

    public double getLeftTrigger() {
        if (joysticksReversed) {
            return driverController.getTriggerAxis(Hand.kRight);
        } else {
            return driverController.getTriggerAxis(Hand.kLeft);
        }
    }

    public double getRightTrigger() {
        if (joysticksReversed) {
            return driverController.getTriggerAxis(Hand.kLeft);
        } else {
            return driverController.getTriggerAxis(Hand.kRight);
        }
    }

    public void setRumble(RUMBLETYPE type, double value) {
        value = value > 1 ? 1 : value;
        switch (type) {
            case DRIVER_LEFT:
                driverController.setRumble(RumbleType.kLeftRumble, value);
            case DRIVER_RIGHT:
                driverController.setRumble(RumbleType.kRightRumble, value);
            case OPERATOR_LEFT:
                operatorController.setRumble(RumbleType.kLeftRumble, value);
            case OPERATOR_RIGHT:
                operatorController.setRumble(RumbleType.kRightRumble, value);
        }
    }

    public void resetRumble() {
        for (RUMBLETYPE type : RUMBLETYPE.values()) {
            setRumble(type, 0);
        }
    }

    public boolean getOpenLoop() {
        return false;
    }

    public boolean getDriveEnabled() {
        return true;
    }

    public boolean getSniperMode() {
        return driverController.getAButton() || driverController.getBButton() || driverController.getBumper(Hand.kLeft) || driverController.getBumper(Hand.kRight);
    }

    public boolean getSniperHigh() {
        return driverController.getBButton() || driverController.getBumper(Hand.kRight);
    }

    public boolean getSniperLow() {
        return driverController.getAButton() || driverController.getBumper(Hand.kLeft);
    }

    public void reverseJoysticks(boolean reverse) {
		joysticksReversed = reverse;
    }

    public double getOperatorStickY() {
        return operatorController.getY(Hand.kLeft);
    }
}