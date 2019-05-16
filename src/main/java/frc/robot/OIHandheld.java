package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.Command;

public class OIHandheld implements OI {
    private boolean joysticksReversed = false;

    // map driver controller to ID 0 and operator controller to ID 1 in driver station
    private Joystick driverController = new Joystick(0);
    private Joystick operatorController = new Joystick(1);
    
    public double getLeftAxis() {
        return 0;
    }

    public double getRightAxis() {
        return 0;
    }

    public double getSingleDriveAxis() {
        return 0;
    }

    public double getHorizDriveAxis() {
        return 0;
    }

    public boolean getOpenLoop() {
        return false;
    }

    public boolean getDriveEnabled() {
        return false;
    }

    public boolean getSniperMode() {
        return false;
    }

    public double getSniperLevel() {
        return 0;
    }

    public void reverseJoysticks(boolean reverse) {
		joysticksReversed = reverse;
    }
    
    public boolean isShiftingEnabled() {
        return false;
    }

    public double getOperatorStickY() {
        return 0;
    }
}