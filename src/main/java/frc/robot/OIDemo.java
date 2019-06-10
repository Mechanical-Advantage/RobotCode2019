/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import frc.robot.commands.ReverseJoysticks;
import frc.robot.commands.ToggleDriveEnabled;
import frc.robot.commands.ToggleOpenLoop;
import frc.robot.commands.ReBotCloseHatchIntake;
import frc.robot.commands.ReBotOpenHatchIntake;
import frc.robot.commands.ReBotRunCargoIntake;
import frc.robot.commands.ReBotRunCargoIntake.IntakeAction;
import frc.robot.commands.ReBotSetIntakeRaised;

/**
 * Uses a single handheld controller with limited functions
 */
public class OIDemo extends OI {
    private boolean joysticksReversed = false;
    private boolean driveEnabled = true;
    private boolean openLoop = true;
    private double driveSpeedScaler = 0.3; // Multiplied by drive speed
    private double elevatorSpeedScaler = 0.3; // Multiplied by elevator speed

    private XboxController controller = new XboxController(0);

    private POVButton joysticksForwards = new POVButton(controller, 0);
    private POVButton joysticksBackwards = new POVButton(controller, 180);
    private JoystickButton toggleDriveEnabled = new JoystickButton(controller, 7); // back button
    private JoystickButton toggleOpenLoop = new JoystickButton(controller, 8); // start button

    private JoystickButton hatchRetract = new JoystickButton(controller, 1); // A button
    private JoystickButton hatchExtend = new JoystickButton(controller, 3); // X button
    private JoystickButton cargoIntake = new JoystickButton(controller, 2); // B button
    private JoystickButton cargoEject = new JoystickButton(controller, 4); // Y button
    private JoystickButton raiseIntake = new JoystickButton(controller, 6); // right bumper
    private JoystickButton lowerIntake = new JoystickButton(controller, 5); // left bumper

    public OIDemo() {
        resetRumble();
        joysticksForwards.whenPressed(new ReverseJoysticks(false));
        joysticksBackwards.whenPressed(new ReverseJoysticks(true));
        toggleDriveEnabled.whenPressed(new ToggleDriveEnabled());
        toggleOpenLoop.whenPressed(new ToggleOpenLoop());

        hatchExtend.whenPressed(new ReBotOpenHatchIntake());
        hatchRetract.whenPressed(new ReBotCloseHatchIntake());
        cargoIntake.whileHeld(new ReBotRunCargoIntake(IntakeAction.INTAKE));
        cargoEject.whileHeld(new ReBotRunCargoIntake(IntakeAction.EJECT));
        raiseIntake.whenPressed(new ReBotSetIntakeRaised(true));
        lowerIntake.whenPressed(new ReBotSetIntakeRaised(false));
    }

    public double getLeftAxis() {
        if (joysticksReversed) {
            return controller.getY(Hand.kRight) * -1 * driveSpeedScaler;
        } else {
            return controller.getY(Hand.kLeft) * driveSpeedScaler;
        }
    }

    public double getRightAxis() {
        if (joysticksReversed) {
            return controller.getY(Hand.kLeft) * -1 * driveSpeedScaler;
        } else {
            return controller.getY(Hand.kRight) * driveSpeedScaler;
        }
    }

    public double getSingleDriveAxisLeft() {
        if (joysticksReversed) {
            return controller.getY(Hand.kLeft) * -1 * driveSpeedScaler;
        } else {
            return controller.getY(Hand.kLeft) * driveSpeedScaler;
        }
    }

    public double getSingleDriveAxisRight() {
        if (joysticksReversed) {
            return controller.getY(Hand.kRight) * -1 * driveSpeedScaler;
        } else {
            return controller.getY(Hand.kRight) * driveSpeedScaler;
        }
    }

    public double getLeftHorizDriveAxis() {
        return controller.getX(Hand.kLeft) * driveSpeedScaler;
    }

    public double getRightHorizDriveAxis() {
        return controller.getX(Hand.kRight) * driveSpeedScaler;
    }

    public void setRumble(OIRumbleType type, double value) {
        value = value > 1 ? 1 : value;
        switch (type) {
        case DRIVER_LEFT:
            controller.setRumble(RumbleType.kLeftRumble, value);
        case DRIVER_RIGHT:
            controller.setRumble(RumbleType.kRightRumble, value);
        default:
        }
    }

    public void resetRumble() {
        setRumble(OIRumbleType.DRIVER_LEFT, 0);
        setRumble(OIRumbleType.DRIVER_RIGHT, 0);
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

    public void reverseJoysticks(boolean reverse) {
        joysticksReversed = reverse;
    }

    public double getLeftOperatorStickY() {
        return (controller.getTriggerAxis(Hand.kRight) - controller.getTriggerAxis(Hand.kLeft)) * elevatorSpeedScaler;
    }

    public double getDeadband() {
        return 0.09;
    }
}
