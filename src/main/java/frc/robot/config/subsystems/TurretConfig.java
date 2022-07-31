package frc.robot.config.subsystems;


public class TurretConfig {

    public static final double inputDeadzone = 0.2;

    public static final double motorMaxOutput = 0.35;
    public static final double clockwiseSpeed = motorMaxOutput * 0.25;
    public static final double counterClockwiseSpeed = -clockwiseSpeed;

    public static final double degreesPerMotorRotation = 4;

    public static final double degreesAtLeft = 90;
    public static final double degreesAtCenter = 180;
    public static final double degreesAtRight = 270;

    public static final float encoderLimit = (float) (90 / degreesPerMotorRotation);
    public static final double nominalMotorRotationsPerSecond = 11000 / 60.0;

    public static final double kP = 0.035;
    public static final double kI = 0.0005;
    public static final double kD = 0.00;

    public static final double moveWithinDegrees = 3;
    public static final long moveCoolDown = 1000;
}
