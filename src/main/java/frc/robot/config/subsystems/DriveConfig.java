package frc.robot.config.subsystems;

public final class DriveConfig {
    public static final int encoderResolution = 360;

	public static final boolean lowGearValue = true;
    public static final boolean highGearValue = !lowGearValue;

    public static final double maxOutput = 1.0;
    public static final double minOutput = 0.6;
    public static final double shiftingMaxOutput = 0.2;
    public static final int shiftCoolDownMillis = 500;

    public static final double joystickDeadzone = 0.15;
    public static final boolean squareInputs = false;
}
