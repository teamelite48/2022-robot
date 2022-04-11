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

    public static final PIDController frontMotorPIDController = new PIDController(0.000015, 0, 0);
    public static final PIDController rearMotorPIDController = new PIDController(0.000015, 0, 0);

    public static final double limelightAngleInDegrees = 35;
    public static final double limelightHeightInFeet = 2.29166666;

    public static final double middleOfVisionTargetInFeet = 8.5675;

    public static final double limelightToVisionTargetInFeet = middleOfVisionTargetInFeet - limelightHeightInFeet;

    public static final double maxDistanceInFeet = 27;

    public static final HashMap<Double, Double> distanceToRpmMap = new HashMap<Double, Double>() {{
        put(0.0, 1100.0);
        put(5.0, 1100.0);
        put(7.0, 1800.0);
        put(8.0, 1900.0);
        put(9.0, 2025.0);
        put(10.0, 2000.0);
        put(11.0, 2050.0);
        put(12.0, 2100.0);
        put(13.0, 2150.0);
        put(14.0, 2200.0);
        put(15.0, 2350.0);
        put(16.0, 2500.0);
        put(17.0, 2900.0);
        put(18.0, 3450.0);
        put(19.0, 3850.0);
        put(20.0, 4100.0);
        put(21.0, 4400.0);
        put(22.0, 4750.0);

    }};
}
