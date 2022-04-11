package frc.robot.config.subsystems;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N7;

public final class DriveConfig {
    public static final int encoderResolution = 360;

	public static final boolean lowGearValue = true;
    public static final boolean highGearValue = !lowGearValue;

    public static final double maxOutput = 1.0;
    public static final double minOutput = 0.6;
    public static final double shiftingMaxOutput = 0.2;
    public static final int shiftCoolDownMillis = 500;

    public static final boolean squareInputs = true;

    public static final double wheelRadiusInMeters = Units.inchesToMeters(2);
    public static final double gearingReduction = 1;

    public static final double ksVolts = 0.75661;
    public static final double kvVoltSecondsPerMeter = 2.2626;
    public static final double kaVoltSecondsSquaredPerMeter = 0.94481;
    public static final double kPDriveVel = 3.5696;

    public static final double kvAngular = 2.3518;
    public static final double kaAngular = 0.39292;

    public static final double trackWidthInMeters = 0.78688;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(trackWidthInMeters);

    public static final double kMaxSpeedMetersPerSecond = .1;
    public static final double kMaxAccelerationMetersPerSecondSquared = .1;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final Vector<N7> enocoderNoise = VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005);
}
