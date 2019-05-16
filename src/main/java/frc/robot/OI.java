package frc.robot;

// Acts as an interface to multiple OI configurations
public interface OI {

    double getLeftAxis();
    double getRightAxis();
    double getSingleDriveAxis();
    double getHorizDriveAxis();
    boolean getOpenLoop();
    boolean getDriveEnabled();
    boolean getSniperMode();
    double getSniperLevel();
    void reverseJoysticks(boolean reverse);
    boolean isShiftingEnabled();
    default boolean isArmEnabled() {
        return false;
    }

    default boolean isTailLocked() {
        return false;
    }

    default double getSliderLevel() {
        return 0;
    }

    default double getLeftTrigger() {
        return 0;
    }

    default double getRightTrigger() {
        return 0;
    }

    double getOperatorStickY();
    default void updateLED(OILED led, boolean state) {}

    static enum OILED {
		MISC_1, MISC_2, MISC_3, INTAKE_RETRACT, INTAKE_ON_OFF, VAC_PICKUP, VAC_TAIL, TOGGLE_LOW, TOGGLE_HIGH,
		JOYSTICK_YELLOW, ARM_ALT, ARM_FLOOR, ARM_CARGO_SHIP, ARM_ROCKET_LOW, ARM_ROCKET_MID, ARM_ROCKET_HIGH, ARM_HOME
	}
}