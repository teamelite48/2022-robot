package frc.robot.config.subsystems;


public final class DriveConfig {

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.66675;

    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.57785;

    public static final int DRIVETRAIN_PIGEON_ID = 5;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 21;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 22;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 31;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(38.7 + 180);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 23;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 24;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 33;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(11.3);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 25;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 26;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 35;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(122.1 + 180);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 27;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 28;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 37;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(24.6);
}
