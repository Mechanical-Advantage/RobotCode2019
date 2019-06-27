package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.buttons.Trigger;
import frc.robot.subsystems.Intake.GamePiece;
import frc.robot.commands.ReverseJoysticks;
import frc.robot.commands.ToggleDriveEnabled;
import frc.robot.commands.ToggleOpenLoop;
import frc.robot.commands.ReBotEjectHatch;
import frc.robot.commands.ReBotCloseHatchIntake;
import frc.robot.commands.ReBotOpenHatchIntake;
import frc.robot.commands.ReBotRunCargoIntake;
import frc.robot.commands.ReBotRunCargoIntake.IntakeAction;
import frc.robot.commands.ReBotSetIntakeRaised;
import frc.robot.commands.ReBotMoveToSetpoint;
import frc.robot.commands.ReBotMoveToSetpoint.OIElevatorPosition;
import frc.robot.commands.ReBotEnableVacuum;
import frc.robot.commands.ReBotDisableVacuum;
import frc.robot.triggers.TriggerPressedTrigger;

public class OIHandheld extends OI {
    private boolean joysticksReversed = false;
    private boolean driveEnabled = true;
    private boolean openLoop = false;

    // map driver controller to ID 0 and operator controller to ID 1 in driver
    // station
    private XboxController driverController = new XboxController(0);
    private XboxController operatorController = new XboxController(1);

    private POVButton joysticksForwards = new POVButton(driverController, 0);
    private POVButton joysticksBackwards = new POVButton(driverController, 180);
    private JoystickButton toggleDriveEnabled = new JoystickButton(driverController, 7); // back button
    private JoystickButton toggleOpenLoop = new JoystickButton(driverController, 8); // start button

    private JoystickButton hatchRetract = new JoystickButton(operatorController, 1); // A button
    private JoystickButton hatchExtend = new JoystickButton(operatorController, 3); // X button
    private JoystickButton cargoIntake = new JoystickButton(operatorController, 2); // B button
    private JoystickButton cargoEject = new JoystickButton(operatorController, 4); // Y button
    private JoystickButton raiseIntake = new JoystickButton(operatorController, 6); // right bumper
    private JoystickButton lowerIntake = new JoystickButton(operatorController, 5); // left bumper

    private POVButton floorSetpoint = new POVButton(operatorController, 0);
    private POVButton shipSetpoint = new POVButton(operatorController, 90);
    private POVButton rocketL1Setpoint = new POVButton(operatorController, 180);
    private POVButton rocketL2Setpoint = new POVButton(operatorController, 270);
    private JoystickButton setHatch = new JoystickButton(operatorController, 7); // back button
    private JoystickButton setCargo = new JoystickButton(operatorController, 8); // start button

    private Trigger enableVacuum = new TriggerPressedTrigger(operatorController, Hand.kRight, 0.6);
    private Trigger disableVacuum = new TriggerPressedTrigger(operatorController, Hand.kLeft, 0.6);

    public OIHandheld() {
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

        floorSetpoint.whenPressed(new ReBotMoveToSetpoint(OIElevatorPosition.FLOOR));
        shipSetpoint.whenPressed(new ReBotMoveToSetpoint(OIElevatorPosition.SHIP));
        rocketL1Setpoint.whenPressed(new ReBotMoveToSetpoint(OIElevatorPosition.ROCKET_L1));
        rocketL2Setpoint.whenPressed(new ReBotMoveToSetpoint(OIElevatorPosition.ROCKET_L2));
        setHatch.whenPressed(new ReBotSetGamepiece(GamePiece.HATCH));
        setCargo.whenPressed(new ReBotSetGamepiece(GamePiece.CARGO));

        enableVacuum.whenActive(new ReBotEnableVacuum());
        disableVacuum.whenActive(new ReBotDisableVacuum());
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

    public void setRumble(OIRumbleType type, double value) {
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
        for (OIRumbleType type : OIRumbleType.values()) {
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

    public double getDeadband() {
        return 0.09;
    }
}