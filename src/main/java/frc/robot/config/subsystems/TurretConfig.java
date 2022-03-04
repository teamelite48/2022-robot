package frc.robot.config.subsystems;

public class TurretConfig {

    public static final double clockwiseSpeed = 0.5;
    public static final double counterClockwiseSpeed = -clockwiseSpeed;

    public static final int encoderLimit = 275;
    public static final double nominalMotorRotationsPerSecond = 11000 / 60.0;

    public static final double kP = 0.026;
    public static final double kI = 0.0003;
    public static final double kD = 0.005;
}
