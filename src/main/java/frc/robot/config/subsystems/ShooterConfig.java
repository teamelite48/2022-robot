package frc.robot.config.subsystems;

import java.util.HashMap;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public final class ShooterConfig {

    public static final double lowRPM = 1100;
    public static final double mediumRPM = 2000;

    public static final double rpmBump = 100;

    public static final SimpleMotorFeedforward frontMotorFeedForward = new SimpleMotorFeedforward(0.69687, 0.11441, 0.009925);
    public static final SimpleMotorFeedforward rearMotorFeedForward = new SimpleMotorFeedforward(0.56945, 0.10931, 0.0027359);

    public static final PIDController frontMotorPIDController = new PIDController(0.00001, 0, 0);
    public static final PIDController rearMotorPIDController = new PIDController(0.00001, 0, 0);

    public static final double limelightAngleInDegrees = 45;
    public static final double limelightHeightInFeet = 2.29166666;

    public static final double middleOfVisionTargetInFeet = 8.5675;

    public static final double limelightToVisionTargetInFeet = middleOfVisionTargetInFeet - limelightHeightInFeet;

    public static final double maxDistanceInFeet = 27;

    public static final HashMap<Integer, Integer> distanceToRpmMap = new HashMap<Integer, Integer>() {{
        put(0, 1800);
        put(1, 1800);
        put(2, 1800);
        put(3, 1800);
        put(4, 1800);
        put(5, 1800);
        put(6, 1800);
        put(7, 1800);
        put(8, 1800);
        put(9, 1850);
        put(10, 1875);
        put(11, 1925);
        put(12, 2050);
        put(13, 2100);
        put(14, 2200);
        put(15, 2300);
        put(16, 2425);
        put(17, 2575);
        put(18, 2850);
        put(19, 3500);
        put(20, 4100);
        put(21, 4400);
        put(22, 4750);
    }};
}
