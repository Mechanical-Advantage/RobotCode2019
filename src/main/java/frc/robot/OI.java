package frc.robot;

// Acts as an interface to multiple OI configurations
public interface OI {
    public double minAcceleration = 0.2; // Minimum total horizontal acceleration before rumbling controller
    public double fullAcceleration = 0.8; // Total horizontal acceleration (g) for full high frequency rumble
    public double lowRumbleFactor = 0.15; // Multiplied by high frequency rumble power to calculate low frequency rumble
                                          // power

    default double getLeftAxis() {
        return 0;
    }

    default double getRightAxis() {
        return 0;
    }

    default double getSingleDriveAxisLeft() {
        return 0;
    }

    default double getSingleDriveAxisRight() {
        return 0;
    }

    default double getLeftHorizDriveAxis() {
        return 0;
    }

    default double getRightHorizDriveAxis() {
        return 0;
    }

    default boolean getOpenLoop() {
        return false;
    }

    default void toggleOpenLoop() {
    };

    default boolean getDriveEnabled() {
        return false;
    }

    default void toggleDriveEnabled() {
    };

    default boolean getSniperMode() {
        return false;
    }

    default double getSniperLevel() {
        return 0;
    }

    default boolean getSniperHigh() {
        return false;
    }

    default boolean getSniperLow() {
        return false;
    }

    default void reverseJoysticks(boolean reverse) {
    }

    default boolean isShiftingEnabled() {
        return false;
    }

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

    default void setRumble(RUMBLETYPE type, double value) {
    }

    default void resetRumble() {
    }

    default double getLeftOperatorStickY() {
        return 0;
    }

    default double getRightOperatorStickY() {
        return 0;
    }

    default void updateLED(OILED led, boolean state) {
    }

    static enum OILED {
        MISC_1, MISC_2, MISC_3, INTAKE_RETRACT, INTAKE_ON_OFF, VAC_PICKUP, VAC_TAIL, TOGGLE_LOW, TOGGLE_HIGH,
        JOYSTICK_YELLOW, ARM_ALT, ARM_FLOOR, ARM_CARGO_SHIP, ARM_ROCKET_LOW, ARM_ROCKET_MID, ARM_ROCKET_HIGH, ARM_HOME
    }

    static enum RUMBLETYPE {
        DRIVER_LEFT, DRIVER_RIGHT, OPERATOR_LEFT, OPERATOR_RIGHT
    }

    static enum OITYPE {
        CONSOLE, HANDHELD
    }
}
