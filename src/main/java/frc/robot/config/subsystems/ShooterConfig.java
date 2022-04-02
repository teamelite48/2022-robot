package frc.robot.config.subsystems;

import java.util.HashMap;

public final class ShooterConfig {

    public static final double lowRPM = 1900;
    public static final double mediumRPM = 4300;
    //public static final double highRPM = 4700;

    public static final double rpmBump = 100;

    public static final double frontkP = 0;
    public static final double frontkI = 0;
    public static final double frontkD = 0;

    public static final double frontks = 0.55338;
    public static final double frontkv = 0.11097;
    public static final double frontka = 0.0066226;

    public static final double rearkP = 0;
    public static final double rearkI = 0;
    public static final double rearkD = 0;

    public static final double rearks = 0.55338;
    public static final double rearkv = 0.11097;
    public static final double rearka = 0.0066226;

    public static final double limelightAngleInDegrees = 45;
    public static final double limelightHeightInFeet = 2;

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
