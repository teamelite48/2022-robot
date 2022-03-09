package frc.robot.config.sysid;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.util.Units;

public final class SysIdConfig {

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

  public static final double kMaxSpeedMetersPerSecond = .15;
  public static final double kMaxAccelerationMetersPerSecondSquared = .15;

  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;


  public static final Vector<N7> enocoderNoise = VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005);
}
