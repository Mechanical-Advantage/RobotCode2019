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
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.DisableArm;
import frc.robot.commands.EjectCargo;
import frc.robot.commands.ExtendSimpleScorer;
import frc.robot.commands.LockBeaverTail;
import frc.robot.commands.LowerPTO;
import frc.robot.commands.ManualArmLightControl;
import frc.robot.commands.ReBotCloseHatchIntake;
import frc.robot.commands.ReBotOpenHatchIntake;
import frc.robot.commands.ReBotRunCargoIntake;
import frc.robot.commands.ReBotRunCargoIntake.IntakeAction;
import frc.robot.commands.ReBotSetIntakeRaised;
import frc.robot.commands.ReleaseTail;
import frc.robot.commands.RetractSimpleScorer;
import frc.robot.commands.RunArmLightIntake;
import frc.robot.commands.RunPTO;
import frc.robot.commands.RunSimpleScorerIntake;
import frc.robot.commands.SetArmLightPosition;
import frc.robot.commands.SetArmLightPosition.ArmLightPosition;
import frc.robot.commands.SetCamera;
import frc.robot.commands.SwitchGear;
import frc.robot.commands.ToggleGear;
import frc.robot.commands.ToggleLevel2Solenoid;
import frc.robot.commands.VacTail;
import frc.robot.commands.ZeroArmFinal;
import frc.robot.commands.ZeroArmInitial;
import frc.robot.subsystems.DriveTrain.DriveGear;
import frc.robot.triggers.JoystickNotCenteredTrigger;
import frc.robot.triggers.MultiButtonTrigger;
import frc.robot.RobotMap;
import frc.robot.RobotMap.RobotType;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OIConsole extends OI {
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

	@SuppressWarnings("unused")
	private static final double elbowMoveAmount = 2;
	private static final double sliderMin = 0.3;
	private static final double cargoIntakeSpeed = 0.15;
	private static final double panelIntakeSpeed = 0.4;
	private static final double panelEjectSpeed = -1;

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
	private Trigger armManualTrigger = new JoystickNotCenteredTrigger(oiController1, AxisType.kY, 0.05);

	private Button level2ClimbFrontToggle = new JoystickButton(leftController, 3);
	private Button level2ClimbRearToggle = new JoystickButton(leftController, 2);

	// private Button armAlt = new JoystickButton(oiController1, 11);
	/*
	 * Arm Heavy: private Button armFloor = new JoystickButton(oiController1, 6);
	 * private Button armCargoShip = new JoystickButton(oiController1, 5);; private
	 * Button armRocketLow = new JoystickButton(oiController1, 4);; private Button
	 * armRocketMid = new JoystickButton(oiController1, 3);; private Button
	 * armRocketHigh = new JoystickButton(oiController1, 2);; private Button
	 * armHomeBackward = new JoystickButton(oiController1, 1);; private Trigger
	 * armFloorPlate = new ButtonNotTrigger(armFloor, armAlt); private Trigger
	 * armFloorCargo = new MultiButtonTrigger(armFloor, armAlt); private Trigger
	 * armCargoShipPlate = new ButtonNotTrigger(armCargoShip, armAlt); private
	 * Trigger armCargoShipCargo = new MultiButtonTrigger(armCargoShip, armAlt);
	 * private Trigger armRocketLowPlate = new ButtonNotTrigger(armRocketLow,
	 * armAlt); private Trigger armRocketLowCargo = new
	 * MultiButtonTrigger(armRocketLow, armAlt); private Trigger armRocketMidPlate =
	 * new ButtonNotTrigger(armRocketMid, armAlt); private Trigger armRocketMidCargo
	 * = new MultiButtonTrigger(armRocketMid, armAlt); private Trigger
	 * armRocketHighPlate = new ButtonNotTrigger(armRocketHigh, armAlt); private
	 * Trigger armRocketHighCargo = new MultiButtonTrigger(armRocketHigh, armAlt);
	 * private Trigger armHome = new ButtonNotTrigger(armHomeBackward, armAlt);
	 * private Trigger armLoadingBackward = new MultiButtonTrigger(armHomeBackward,
	 * armAlt);
	 */
	// Arm Light:
	private Button armLoading = new JoystickButton(oiController1, 6);
	private Button armCargoShip = new JoystickButton(oiController1, 5);
	private Button armRocketLow = new JoystickButton(oiController1, 4);
	private Button armFloor = new JoystickButton(oiController1, 3);
	private Button armCamera = new JoystickButton(oiController1, 2); // Rocket high is unused because arm light cannot
																		// reach it
	private Button armHome = new JoystickButton(oiController1, 1);
	private Button armUp = new JoystickButton(oiController1, 11);
	// private Trigger armLoadingPlate = new MultiButtonTrigger(armLoading, armAlt);
	// private Trigger armLoadingCargo = new ButtonNotTrigger(armLoading, armAlt);
	// private Trigger armCargoShipPlate = new MultiButtonTrigger(armCargoShip,
	// armAlt);
	// private Trigger armCargoShipCargo = new ButtonNotTrigger(armCargoShip,
	// armAlt);
	// private Trigger armRocketLowPlate = new MultiButtonTrigger(armRocketLow,
	// armAlt);
	// private Trigger armRocketLowCargo = new ButtonNotTrigger(armRocketLow,
	// armAlt);
	// private Trigger armRocketMidPlate = new MultiButtonTrigger(armRocketMid,
	// armAlt);
	// private Trigger armRocketMidCargo = new ButtonNotTrigger(armRocketMid,
	// armAlt);
	// private Trigger armRocketHighPlate = new ButtonNotTrigger(armRocketHigh,
	// armAlt);
	// private Trigger armRocketHighCargo = new MultiButtonTrigger(armRocketHigh,
	// armAlt);

	// private Button elbowUp = new JoystickButton(oiController1, 9);
	// private Button elbowDown = new JoystickButton(oiController1, 10);

	// private Button vacPickup = new JoystickButton(oiController2, 3); // Suction
	// pickup no longer used

	private Button tailLock = new JoystickButton(oiController1, 12); // Is a switch
	private Trigger releaseTail = new MultiButtonTrigger(new JoystickButton(leftController, 7),
			new JoystickButton(rightController, 10));
	private Button tailVac = new JoystickButton(oiController2, 4);
	private Button runPTO = new JoystickButton(rightController, 11);
	private Button lowerPTO = new JoystickButton(leftController, 6);

	private Button extendSimpleScorer = new JoystickButton(oiController2, 2);
	private Button retractSimpleScorer = new JoystickButton(oiController2, 1);

	private Button intakePanel = new JoystickButton(oiController1, 10);
	private Button ejectPanel = new JoystickButton(oiController1, 9);

	private Button intakeCargo = new JoystickButton(oiController2, 3);
	private Button ejectCargo = new JoystickButton(oiController2, 5);

	// ReBot buttons
	private Trigger rebotHatchRetract = new JoystickButton(oiController2, 1);
	private Trigger rebotHatchExtend = new JoystickButton(oiController2, 2);
	private Trigger rebotCargoIntake = new JoystickButton(oiController1, 10);
	private Trigger rebotCargoEject = new JoystickButton(oiController1, 9);
	private Trigger rebotRaiseIntake = new JoystickButton(oiController2, 4);
	private Trigger rebotLowerIntake = new JoystickButton(oiController2, 3);

	NetworkTable ledTable;
	NetworkTableEntry ledEntry;

	public OIConsole() {
		ledTable = NetworkTableInstance.getDefault().getTable("LEDs");
		ledEntry = ledTable.getEntry("OI LEDs");

		ledEntry.setBooleanArray(new boolean[] { false, false, false, false, false, false, false, false, false, false,
				false, false, false, false, false, false, false });

		if (RobotMap.robot == RobotType.ROBOT_REBOT) {
			rebotHatchExtend.whenActive(new ReBotOpenHatchIntake());
			rebotHatchRetract.whenActive(new ReBotCloseHatchIntake());
			rebotCargoIntake.whileActive(new ReBotRunCargoIntake(IntakeAction.INTAKE));
			rebotCargoEject.whileActive(new ReBotRunCargoIntake(IntakeAction.EJECT));
			rebotRaiseIntake.whenActive(new ReBotSetIntakeRaised(true));
			rebotLowerIntake.whenActive(new ReBotSetIntakeRaised(false));
		} else {
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
			armManualTrigger.whenActive(new ManualArmLightControl());

			/*
			 * Arm Heavy: armFloorPlate.whenActive(new
			 * SetArmPositions(ArmPosition.FLOOR_PLATE)); armFloorCargo.whenActive(new
			 * SetArmPositions(ArmPosition.FLOOR_CARGO)); armCargoShipPlate.whenActive(new
			 * SetArmPositions(ArmPosition.CARGOSHIP_PLATE));
			 * armCargoShipCargo.whenActive(new
			 * SetArmPositions(ArmPosition.CARGOSHIP_CARGO));
			 * armRocketLowPlate.whenActive(new
			 * SetArmPositions(ArmPosition.ROCKET_LO_PLATE));
			 * armRocketLowCargo.whenActive(new
			 * SetArmPositions(ArmPosition.ROCKET_LO_CARGO));
			 * armRocketMidPlate.whenActive(new
			 * SetArmPositions(ArmPosition.ROCKET_MID_PLATE));
			 * armRocketMidCargo.whenActive(new
			 * SetArmPositions(ArmPosition.ROCKET_MID_CARGO));
			 * armRocketHighPlate.whenActive(new
			 * SetArmPositions(ArmPosition.ROCKET_HI_PLATE));
			 * armRocketHighCargo.whenActive(new
			 * SetArmPositions(ArmPosition.ROCKET_HI_CARGO)); armHome.whenActive(new
			 * SetArmPositions(ArmPosition.HOME)); armLoadingBackward.whenActive(new
			 * SetArmPositions(ArmPosition.LOADING_PICKUP_BACKWARDS));
			 */
			// Arm Light:
			armHome.whenActive(new SetArmLightPosition(ArmLightPosition.HOME));
			armLoading.whenActive(new SetArmLightPosition(ArmLightPosition.LOADING_CARGO));
			// armLoadingPlate.whenActive(new
			// SetArmLightPosition(ArmLightPosition.LOADING_PLATE));
			armCargoShip.whenActive(new SetArmLightPosition(ArmLightPosition.CARGOSHIP_CARGO));
			// armCargoShipPlate.whenActive(new
			// SetArmLightPosition(ArmLightPosition.CARGOSHIP_PLATE));
			armRocketLow.whenActive(new SetArmLightPosition(ArmLightPosition.ROCKET_LO_CARGO));
			// armRocketLowPlate.whenActive(new
			// SetArmLightPosition(ArmLightPosition.ROCKET_LO_PLATE));
			// armRocketMidCargo.whenActive(new
			// SetArmLightPosition(ArmLightPosition.ROCKET_MID_CARGO));
			// armRocketMidPlate.whenActive(new
			// SetArmLightPosition(ArmLightPosition.ROCKET_MID_PLATE));
			armCamera.whenActive(new SetArmLightPosition(ArmLightPosition.CAMERA));
			armFloor.whenActive(new SetArmLightPosition(ArmLightPosition.FLOOR_CARGO));
			armUp.whenActive(new SetArmLightPosition(ArmLightPosition.UP));

			// Command elbowUpCommand = new MoveElbowLight(elbowMoveAmount);
			// Command elbowDownCommand = new MoveElbowLight(elbowMoveAmount*-1);
			// elbowUp.whenPressed(elbowUpCommand);
			// elbowUp.whenReleased(new CancelCommand(elbowUpCommand));
			// elbowDown.whenPressed(elbowDownCommand);
			// elbowDown.whenReleased(new CancelCommand(elbowDownCommand));

			// vacPickup.whenPressed(new VacPickupToggle());

			tailLock.whileHeld(new LockBeaverTail());
			releaseTail.whenActive(new ReleaseTail());
			tailVac.toggleWhenPressed(new VacTail());
			runPTO.toggleWhenPressed(new RunPTO());
			lowerPTO.whileHeld(new LowerPTO());

			extendSimpleScorer.whenPressed(new ExtendSimpleScorer());
			retractSimpleScorer.whenPressed(new RetractSimpleScorer());

			intakePanel.whileHeld(new RunSimpleScorerIntake(panelIntakeSpeed));
			// ejectPanel.whileHeld(new EjectPanel());
			// ejectPanel.whenReleased(new RetractSimpleScorer());
			ejectPanel.whileHeld(new RunSimpleScorerIntake(panelEjectSpeed));

			intakeCargo.whileHeld(new RunArmLightIntake(cargoIntakeSpeed));
			ejectCargo.whileHeld(new EjectCargo());

			level2ClimbFrontToggle.whenPressed(new ToggleLevel2Solenoid(true));
			level2ClimbRearToggle.whenPressed(new ToggleLevel2Solenoid(false));
		}
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
	public double getSingleDriveAxisLeft() {
		if (joysticksReversed) {
			return leftController.getRawAxis(1) * -1;
		} else {
			return leftController.getRawAxis(1);
		}
	}

	public double getSingleDriveAxisRight() {
		if (joysticksReversed) {
			return rightController.getRawAxis(1) * -1;
		} else {
			return rightController.getRawAxis(1);
		}
	}

	public double getLeftHorizDriveAxis() {
		return leftController.getRawAxis(0);
	}

	public double getRightHorizDriveAxis() {
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

	public double getSliderLevel() {
		return Robot.map(oiController2.getRawAxis(1), -1, 1, sliderMin, 1);
	}

	public double getArmLightStick() {
		return oiController1.getY();
	}

	public double getElevatorStick() {
		return oiController1.getY() * -1;
	}

	public double getDeadband() {
		return 0.05;
	}

	public void updateLED(OILED led, boolean state) {
		boolean[] array = ledTable.getEntry("OI LEDs").getBooleanArray(new boolean[] { false, false, false, false,
				false, false, false, false, false, false, false, false, false, false, false, false, false });
		array[led.ordinal()] = state;
		ledEntry.setBooleanArray(array);
	}
}
