package frc.robot.config.sysid;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.util.Units;

public final class SysIdConfig {

  public static final double wheelRadiusInMeters = Units.inchesToMeters(2);
  public static final double gearingReduction = 1;

  public static final double ksVolts = 0.22;
  public static final double kvVoltSecondsPerMeter = 1.98;
  public static final double kaVoltSecondsSquaredPerMeter = 0.2;
  public static final double kPDriveVel = 8.5;

  public static final double kvLinear = 1.98;
  public static final double kaLinear = 0.2;
  public static final double kvAngular = 1.5;
  public static final double kaAngular = 0.3;

  public static final double trackWidthInMeters = 0.7112;
  public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(trackWidthInMeters);

  public static final int kMaxSpeedMetersPerSecond = 3;
  public static final int kMaxAccelerationMetersPerSecondSquared = 3;

  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;


  public static final Vector<N7> enocoderNoise = VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005);
}
