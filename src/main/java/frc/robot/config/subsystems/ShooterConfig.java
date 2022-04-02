package frc.robot.config.subsystems;

import java.util.HashMap;

public final class ShooterConfig {

    public static final double lowRPM = 1300;
    public static final double mediumRPM = 2800;
    //public static final double highRPM = 4700;

    public static final double rpmBump = 100;

    public static final double frontkP = 0.00015;
    public static final double frontkI = 0;
    public static final double frontkD = 0;

    public static final double frontks = 0.69687;
    public static final double frontkv = 0.11441;
    public static final double frontka = 0.009925;

    public static final double rearkP = 0.00015;
    public static final double rearkI = 0;
    public static final double rearkD = 0;

    public static final double rearks = 0.56945;
    public static final double rearkv = 0.10931;
    public static final double rearka = 0.0027359;

    public static final double limelightAngleInDegrees = 45;
    public static final double limelightHeightInFeet = 2.29166666;

    public static final double middleOfVisionTargetInFeet = 8.5675;

    public static final double limelightToVisionTarget = middleOfVisionTargetInFeet - limelightHeightInFeet;

    public static final double maxDistanceInFeet = 27;

    public static final HashMap<Integer, Integer> distanceToRPMMap = new HashMap<Integer, Integer>() {{

        put(0, 0);
        put(1, 1000);
        put(2, 2000);
        put(3, 1000);
        put(4, 2000);
        put(5, 1000);
        put(6, 2000);

    }};
    
}
