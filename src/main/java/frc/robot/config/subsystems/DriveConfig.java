package frc.robot.config.subsystems;

public final class DriveConfig {
    public static final int encoderResolution = 360;

	public static final boolean lowGearValue = true;
    public static final boolean highGearValue = !lowGearValue;

    public static final double normalMaxOutput = 0.5;
    public static final double shiftingMaxOutput = 0.2;
    public static final int shiftCoolDownMillis = 500;
}
