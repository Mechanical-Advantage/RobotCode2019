package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Intake.GamePiece;
import frc.robot.commands.ReverseJoysticks;
import frc.robot.commands.ToggleDriveEnabled;
import frc.robot.commands.ToggleOpenLoop;
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
import frc.robot.triggers.DemoSwitchingTrigger;

public class OIHandheld extends OI {
    private boolean joysticksReversed = false;
    private boolean driveEnabled = true;
    private boolean openLoop = true;
    private double demoDriveSpeedScaler = 0.6; // Multiplied by drive speed (demo mode only)
    private double demoElevatorSpeedScaler = 0.7; // Multiplied by elevator speed (demo mode only)

    // map driver controller to ID 0 and operator controller to ID 1 in driver
    // station
    private XboxController driverController = new XboxController(0);
    private XboxController operatorController = new XboxController(1);

    private POVButton joysticksForwards = new POVButton(driverController, 0);
    private POVButton joysticksBackwards = new POVButton(driverController, 180);
    private JoystickButton toggleDriveEnabled = new JoystickButton(driverController, 7); // back button
    private JoystickButton toggleOpenLoop = new JoystickButton(driverController, 8); // start button

    private Trigger hatchRetract = new DemoSwitchingTrigger(new JoystickButton(operatorController, 1),
            new JoystickButton(driverController, 1)); // A button
    private Trigger hatchExtend = new DemoSwitchingTrigger(new JoystickButton(operatorController, 3),
            new JoystickButton(driverController, 3)); // X button
    private Trigger cargoIntake = new DemoSwitchingTrigger(new JoystickButton(operatorController, 2),
            new JoystickButton(driverController, 2)); // B button
    private Trigger cargoEject = new DemoSwitchingTrigger(new JoystickButton(operatorController, 4),
            new JoystickButton(driverController, 4)); // Y button
    private Trigger raiseIntake = new DemoSwitchingTrigger(new JoystickButton(operatorController, 6),
            new JoystickButton(driverController, 6)); // right bumper
    private Trigger lowerIntake = new DemoSwitchingTrigger(new JoystickButton(operatorController, 5),
            new JoystickButton(driverController, 5)); // left bumper

    private Trigger floorSetpoint = new DemoSwitchingTrigger(new POVButton(operatorController, 0), null);
    private Trigger shipSetpoint = new DemoSwitchingTrigger(new POVButton(operatorController, 90), null);
    private Trigger rocketL1Setpoint = new DemoSwitchingTrigger(new POVButton(operatorController, 180), null);
    private Trigger rocketL2Setpoint = new DemoSwitchingTrigger(new POVButton(operatorController, 270), null);
    private Trigger setHatch = new DemoSwitchingTrigger(new JoystickButton(operatorController, 7), null); // back button
    private Trigger setCargo = new DemoSwitchingTrigger(new JoystickButton(operatorController, 8), null); // start
                                                                                                          // button

    private Trigger enableVacuum = new DemoSwitchingTrigger(
            new TriggerPressedTrigger(operatorController, Hand.kRight, 0.6), null);
    private Trigger disableVacuum = new DemoSwitchingTrigger(
            new TriggerPressedTrigger(operatorController, Hand.kLeft, 0.6), null);

    public OIHandheld() {
        resetRumble();
        joysticksForwards.whenPressed(new ReverseJoysticks(false));
        joysticksBackwards.whenPressed(new ReverseJoysticks(true));
        toggleDriveEnabled.whenPressed(new ToggleDriveEnabled());
        toggleOpenLoop.whenPressed(new ToggleOpenLoop());

        hatchExtend.whenActive(new ReBotOpenHatchIntake());
        hatchRetract.whenActive(new ReBotCloseHatchIntake());
        cargoIntake.whileActive(new ReBotRunCargoIntake(IntakeAction.INTAKE));
        cargoEject.whileActive(new ReBotRunCargoIntake(IntakeAction.EJECT));
        raiseIntake.whenActive(new ReBotSetIntakeRaised(true));
        lowerIntake.whenActive(new ReBotSetIntakeRaised(false));

        floorSetpoint.whenActive(new ReBotMoveToSetpoint(OIElevatorPosition.FLOOR));
        shipSetpoint.whenActive(new ReBotMoveToSetpoint(OIElevatorPosition.SHIP));
        rocketL1Setpoint.whenActive(new ReBotMoveToSetpoint(OIElevatorPosition.ROCKET_L1));
        rocketL2Setpoint.whenActive(new ReBotMoveToSetpoint(OIElevatorPosition.ROCKET_L2));
        setHatch.whenActive(new ReBotSetGamepiece(GamePiece.HATCH));
        setCargo.whenActive(new ReBotSetGamepiece(GamePiece.CARGO));

        enableVacuum.whenActive(new ReBotEnableVacuum());
        disableVacuum.whenActive(new ReBotDisableVacuum());
    }

    public double getLeftAxis() {
        if (joysticksReversed) {
            return driverController.getY(Hand.kRight) * -1 * getDriveSpeedScaler();
        } else {
            return driverController.getY(Hand.kLeft) * getDriveSpeedScaler();
        }
    }

    public double getRightAxis() {
        if (joysticksReversed) {
            return driverController.getY(Hand.kLeft) * -1 * getDriveSpeedScaler();
        } else {
            return driverController.getY(Hand.kRight) * getDriveSpeedScaler();
        }
    }

    public double getSingleDriveAxisLeft() {
        if (joysticksReversed) {
            return driverController.getY(Hand.kLeft) * -1 * getDriveSpeedScaler();
        } else {
            return driverController.getY(Hand.kLeft) * getDriveSpeedScaler();
        }
    }

    public double getSingleDriveAxisRight() {
        if (joysticksReversed) {
            return driverController.getY(Hand.kRight) * -1 * getDriveSpeedScaler();
        } else {
            return driverController.getY(Hand.kRight) * getDriveSpeedScaler();
        }
    }

    public double getLeftHorizDriveAxis() {
        return driverController.getX(Hand.kLeft) * getDriveSpeedScaler();
    }

    public double getRightHorizDriveAxis() {
        return driverController.getX(Hand.kRight) * getDriveSpeedScaler();
    }

    public double getLeftTrigger() {
        if (getDemoMode()) {
            return 0;
        } else {
            if (joysticksReversed) {
                return driverController.getTriggerAxis(Hand.kRight);
            } else {
                return driverController.getTriggerAxis(Hand.kLeft);
            }
        }
    }

    public double getRightTrigger() {
        if (getDemoMode()) {
            return 0;
        } else {
            if (joysticksReversed) {
                return driverController.getTriggerAxis(Hand.kLeft);
            } else {
                return driverController.getTriggerAxis(Hand.kRight);
            }
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
            if (!getDemoMode()) {
                operatorController.setRumble(RumbleType.kLeftRumble, value);
            }
        case OPERATOR_RIGHT:
            if (!getDemoMode()) {
                operatorController.setRumble(RumbleType.kRightRumble, value);
            }
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
        if (getDemoMode()) {
            return false;
        } else {
            return driverController.getAButton() || driverController.getBButton()
                    || driverController.getBumper(Hand.kLeft) || driverController.getBumper(Hand.kRight);
        }
    }

    public boolean getSniperHigh() {
        if (getDemoMode()) {
            return false;
        } else {
            return driverController.getBButton() || driverController.getBumper(Hand.kRight);
        }
    }

    public boolean getSniperLow() {
        if (getDemoMode()) {
            return false;
        } else {
            return driverController.getAButton() || driverController.getBumper(Hand.kLeft);
        }
    }

    public void reverseJoysticks(boolean reverse) {
        joysticksReversed = reverse;
    }

    public double getLeftOperatorStickY() {
        if (getDemoMode()) {
            return (driverController.getTriggerAxis(Hand.kLeft) - driverController.getTriggerAxis(Hand.kRight))
                    * demoElevatorSpeedScaler;
        } else {
            return operatorController.getY(Hand.kLeft);
        }
    }

    public double getRightOperatorStickY() {
        if (getDemoMode()) {
            return 0;
        } else {
            return operatorController.getY(Hand.kRight);
        }
    }

    public double getDeadband() {
        return 0.09;
    }

    private double getDriveSpeedScaler() {
        return getDemoMode() ? demoDriveSpeedScaler : 1;
    }

    public boolean getDemoMode() {
        return SmartDashboard.getBoolean("Demo Controls", false);
    }
}