package frc.robot.config.subsystems;

public class TurretConfig {

    public static final double inputDeadzone = 0.2;

    public static final double motorMaxOutput = 0.35;
    public static final double clockwiseSpeed = motorMaxOutput;
    public static final double counterClockwiseSpeed = -clockwiseSpeed;

    public static final int encoderLimit = 6;
    public static final double nominalMotorRotationsPerSecond = 11000 / 60.0;

    public static final double kP = 0.04;
    public static final double kI = 0.0003;
    public static final double kD = 0.005;
}
