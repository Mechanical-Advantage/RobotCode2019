/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.DisableArm;
import frc.robot.commands.ExtendSimpleScorer;
import frc.robot.commands.LockBeaverTail;
import frc.robot.commands.MoveElbow;
import frc.robot.commands.ReleaseTail;
import frc.robot.commands.RetractSimpleScorer;
import frc.robot.commands.RunPTO;
import frc.robot.commands.SetArmPositions;
import frc.robot.commands.SetArmPositions.ArmPosition;
import frc.robot.commands.SetCamera;
import frc.robot.commands.SwitchGear;
import frc.robot.commands.ToggleGear;
import frc.robot.commands.VacPickupToggle;
import frc.robot.commands.VacTail;
import frc.robot.commands.ZeroArmFinal;
import frc.robot.commands.ZeroArmInitial;
import frc.robot.subsystems.DriveTrain.DriveGear;
import frc.robot.triggers.ButtonNotTrigger;
import frc.robot.triggers.MultiButtonTrigger;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());

	private static final double elbowMoveAmount = 2;

	private boolean joysticksReversed = false;

	// map left stick to ID 0 and right to ID 1 in driver station
	private Joystick leftController = new Joystick(0);
	private Joystick rightController = new Joystick(1);
	private Joystick oiController1 = new Joystick(2);
	private Joystick oiController2 = new Joystick(3);

	private Button frontCameraButton = new JoystickButton(rightController, 3);
	private Button rearCameraButton = new JoystickButton(rightController, 2);
	@SuppressWarnings("unused")
	private Button joysticksForward = new JoystickButton(leftController, 3);
	@SuppressWarnings("unused")
	private Button joysticksBackward = new JoystickButton(leftController, 2);
	private Button sniperMode = new JoystickButton(rightController, 1);
	private Button toggleGear = new JoystickButton(leftController, 1);
	private Button openLoopDrive = new JoystickButton(oiController2, 10);
	private Button driveDisableSwitch = new JoystickButton(oiController2, 9);
	private Button shiftDisableSwitch = new JoystickButton(oiController2, 8);
	private Button highGear = new JoystickButton(leftController, 5);
	private Button lowGear = new JoystickButton(leftController, 4);

	private Button armDisable = new JoystickButton(oiController2, 8);
	private Button armZeroInitial = new JoystickButton(oiController2, 7);
	private Button armZeroFinal = new JoystickButton(oiController2, 6);

	private Button armAlt = new JoystickButton(oiController1, 11);
	private Button armFloor = new JoystickButton(oiController1, 6);
	private Button armCargoShip = new JoystickButton(oiController1, 5);;
	private Button armRocketLow = new JoystickButton(oiController1, 4);;
	private Button armRocketMid = new JoystickButton(oiController1, 3);;
	private Button armRocketHigh = new JoystickButton(oiController1, 2);;
	private Button armHomeBackward = new JoystickButton(oiController1, 1);;
	private Trigger armFloorPlate = new ButtonNotTrigger(armFloor, armAlt);
	private Trigger armFloorCargo = new MultiButtonTrigger(armFloor, armAlt);
	private Trigger armCargoShipPlate = new ButtonNotTrigger(armCargoShip, armAlt);
	private Trigger armCargoShipCargo = new MultiButtonTrigger(armCargoShip, armAlt);
	private Trigger armRocketLowPlate = new ButtonNotTrigger(armRocketLow, armAlt);
	private Trigger armRocketLowCargo = new MultiButtonTrigger(armRocketLow, armAlt);
	private Trigger armRocketMidPlate = new ButtonNotTrigger(armRocketMid, armAlt);
	private Trigger armRocketMidCargo = new MultiButtonTrigger(armRocketMid, armAlt);
	private Trigger armRocketHighPlate = new ButtonNotTrigger(armRocketHigh, armAlt);
	private Trigger armRocketHighCargo = new MultiButtonTrigger(armRocketHigh, armAlt);
	private Trigger armHome = new ButtonNotTrigger(armHomeBackward, armAlt);
	private Trigger armLoadingBackward = new MultiButtonTrigger(armHomeBackward, armAlt);

	private Button elbowUp = new JoystickButton(oiController1, 9);
	private Button elbowDown = new JoystickButton(oiController1, 10);

	private Button vacPickup = new JoystickButton(oiController2, 3);

	private Button tailLock = new JoystickButton(oiController1, 12); // Is a switch
	private Trigger releaseTail = new MultiButtonTrigger(new JoystickButton(leftController, 7),
		new JoystickButton(rightController, 10));
	private Button tailVac = new JoystickButton(oiController2, 4);
	private Button runPTO = new JoystickButton(rightController, 11);

	private Button extendSimpleScorer = new JoystickButton(oiController2, 2);
	private Button retractSimpleScorer = new JoystickButton(oiController2, 1);

	NetworkTable ledTable;
	NetworkTableEntry ledEntry;

	public OI() {
		ledTable = NetworkTableInstance.getDefault().getTable("LEDs");
		ledEntry = ledTable.getEntry("OI LEDs");

		frontCameraButton.whenPressed(new SetCamera(true));
		rearCameraButton.whenPressed(new SetCamera(false));
		// joysticksForward.whenPressed(new SetCamera(true));
		// joysticksBackward.whenPressed(new SetCamera(false));
		// joysticksForward.whenPressed(new ReverseJoysticks(false));
		// joysticksBackward.whenPressed(new ReverseJoysticks(true));
		highGear.whenPressed(new SwitchGear(DriveGear.HIGH));
		lowGear.whenPressed(new SwitchGear(DriveGear.LOW));
		toggleGear.whenPressed(new ToggleGear());

		armDisable.whenPressed(new DisableArm());
		Command armZeroInitialCommand = new ZeroArmInitial();
		armZeroInitial.whenPressed(armZeroInitialCommand);
		armZeroInitial.whenReleased(new CancelCommand(armZeroInitialCommand));
		Command armZeroFinalCommand = new ZeroArmFinal();
		armZeroFinal.whenPressed(armZeroFinalCommand);
		armZeroFinal.whenReleased(new CancelCommand(armZeroFinalCommand));

		armFloorPlate.whenActive(new SetArmPositions(ArmPosition.FLOOR_PLATE));
		armFloorCargo.whenActive(new SetArmPositions(ArmPosition.FLOOR_CARGO));
		armCargoShipPlate.whenActive(new SetArmPositions(ArmPosition.CARGOSHIP_PLATE));
		armCargoShipCargo.whenActive(new SetArmPositions(ArmPosition.CARGOSHIP_CARGO));
		armRocketLowPlate.whenActive(new SetArmPositions(ArmPosition.ROCKET_LO_PLATE));
		armRocketLowCargo.whenActive(new SetArmPositions(ArmPosition.ROCKET_LO_CARGO));
		armRocketMidPlate.whenActive(new SetArmPositions(ArmPosition.ROCKET_MID_PLATE));
		armRocketMidCargo.whenActive(new SetArmPositions(ArmPosition.ROCKET_MID_CARGO));
		armRocketHighPlate.whenActive(new SetArmPositions(ArmPosition.ROCKET_HI_PLATE));
		armRocketHighCargo.whenActive(new SetArmPositions(ArmPosition.ROCKET_HI_CARGO));
		armHome.whenActive(new SetArmPositions(ArmPosition.HOME));
		armLoadingBackward.whenActive(new SetArmPositions(ArmPosition.LOADING_PICKUP_BACKWARDS));

		Command elbowUpCommand = new MoveElbow(elbowMoveAmount);
		Command elbowDownCommand = new MoveElbow(elbowMoveAmount*-1);
		elbowUp.whenPressed(elbowUpCommand);
		elbowUp.whenReleased(new CancelCommand(elbowUpCommand));
		elbowDown.whenPressed(elbowDownCommand);
		elbowDown.whenReleased(new CancelCommand(elbowDownCommand));

		vacPickup.whenPressed(new VacPickupToggle());

		tailLock.whenPressed(new LockBeaverTail());
		releaseTail.whenActive(new ReleaseTail());
		tailVac.toggleWhenPressed(new VacTail());
		runPTO.toggleWhenPressed(new RunPTO());

		extendSimpleScorer.whenPressed(new ExtendSimpleScorer());
		retractSimpleScorer.whenPressed(new RetractSimpleScorer());

		ledEntry.setBooleanArray(new boolean[] { false, false, false, false,
			false, false, false, false, false, false, false, false, false, false, false, false, false });
	}

	public double getLeftAxis() {
		if (joysticksReversed) {
			return rightController.getRawAxis(1) * -1;
		} else {
			return leftController.getRawAxis(1);
		}
	}

	public double getRightAxis() {
		if (joysticksReversed) {
			return leftController.getRawAxis(1) * -1;
		} else {
			return rightController.getRawAxis(1);
		}
	}

	// reversing the joysticks should not change which joystick to use for straight
	// drive, use
	// different function to make that correct
	// Note: Brian is left-handed
	public double getSingleDriveAxis() {
		if (joysticksReversed) {
			return leftController.getRawAxis(1) * -1;
		} else {
			return leftController.getRawAxis(1);
		}
	}

	public double getHorizDriveAxis() {
		return rightController.getRawAxis(0);
	}

	public boolean getOpenLoop() {
		return openLoopDrive.get();
	}

	public boolean getDriveEnabled() {
		return !driveDisableSwitch.get();
	}

	public boolean getSniperMode() {
		return sniperMode.get();
	}

	public double getSniperLevel() {
		double sniperLimit = 0.5;
		return (1 - ((rightController.getRawAxis(2) + 1) / 2)) * sniperLimit; // control returns -1 to 1, scale to 0 to
																				// 1, subtract from 1 so 1 is up
	}

	public void reverseJoysticks(boolean reverse) {
		joysticksReversed = reverse;
	}

	public boolean isShiftingEnabled() {
		return !shiftDisableSwitch.get();
	}

	public boolean isArmEnabled() {
		return !armDisable.get();
	}

	public boolean isTailLocked() {
		return tailLock.get();
	}

	public void updateLED(OILED led, boolean state) {
		boolean[] array = ledTable.getEntry("OI LEDs").getBooleanArray(new boolean[] { false, false, false, false,
				false, false, false, false, false, false, false, false, false, false, false, false, false });
		array[led.ordinal()] = state;
		ledEntry.setBooleanArray(array);
	}

	public enum OILED {
		MISC_1, MISC_2, MISC_3, INTAKE_RETRACT, INTAKE_ON_OFF, VAC_PICKUP, VAC_TAIL, TOGGLE_LOW, TOGGLE_HIGH,
		JOYSTICK_YELLOW, ARM_ALT, ARM_FLOOR, ARM_CARGO_SHIP, ARM_ROCKET_LOW, ARM_ROCKET_MID, ARM_ROCKET_HIGH, ARM_HOME
	}
}
