package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;

import frc.robot.commands.ReverseJoysticks;
import frc.robot.commands.ToggleDriveEnabled;
import frc.robot.commands.ToggleOpenLoop;

public class OIHandheld implements OI {
    private boolean joysticksReversed = false;
    private boolean driveEnabled = false;
    private boolean openLoop = false;

    // map driver controller to ID 0 and operator controller to ID 1 in driver
    // station
    private XboxController driverController = new XboxController(0);
    private XboxController operatorController = new XboxController(1);

    private POVButton joysticksForwards = new POVButton(driverController, 0);
    private POVButton joysticksBackwards = new POVButton(driverController, 180);
    private JoystickButton driverToggleDriveEnabled = new JoystickButton(driverController, 7);
    private JoystickButton driverToggleOpenLoop = new JoystickButton(driverController, 8);
    private JoystickButton operatorToggleDriveEnabled = new JoystickButton(operatorController, 7);
    private JoystickButton operatorToggleOpenLoop = new JoystickButton(operatorController, 8);

    public OIHandheld() {
        resetRumble();
        joysticksForwards.whenPressed(new ReverseJoysticks(false));
        joysticksBackwards.whenPressed(new ReverseJoysticks(true));
        driverToggleDriveEnabled.whenPressed(new ToggleDriveEnabled());
        driverToggleOpenLoop.whenPressed(new ToggleOpenLoop());
        operatorToggleDriveEnabled.whenPressed(new ToggleDriveEnabled());
        operatorToggleOpenLoop.whenPressed(new ToggleOpenLoop());
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

    public double getSingleDriveAxisLeft() {
        if (joysticksReversed) {
            return driverController.getY(Hand.kLeft) * -1;
        } else {
            return driverController.getY(Hand.kLeft);
        }
    }

    public double getSingleDriveAxisRight() {
        if (joysticksReversed) {
            return driverController.getY(Hand.kRight) * -1;
        } else {
            return driverController.getY(Hand.kRight);
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
        return openLoop;
    }

    public void toggleOpenLoop() {
        openLoop = !openLoop;
    }

    public boolean getDriveEnabled() {
        return driveEnabled;
    }

    public void toggleDriveEnabled() {
        driveEnabled = !driveEnabled;
    }

    public boolean getSniperMode() {
        return driverController.getAButton() || driverController.getBButton() || driverController.getBumper(Hand.kLeft)
                || driverController.getBumper(Hand.kRight);
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

    public double getLeftOperatorStickY() {
        return operatorController.getY(Hand.kLeft);
    }

    public double getRightOperatorStickY() {
        return operatorController.getY(Hand.kRight);
    }
}