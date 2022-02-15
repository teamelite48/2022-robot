package frc.robot.config.pathfollowing;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import frc.robot.config.subsystems.DriveConfig;

public final class PathFollowingConfig {

  // TODO: run system id tool and get real values

  public static final double ksVolts = 0.22;
  public static final double kvVoltSecondsPerMeter = 1.98;
  public static final double kaVoltSecondsSquaredPerMeter = 0.2;
  public static final double kPDriveVel = 8.5;

  public static final double trackWidthInMeters = 0.7112;
  public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(trackWidthInMeters);

  public static final int kMaxSpeedMetersPerSecond = 3;
  public static final int kMaxAccelerationMetersPerSecondSquared = 3;

  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;

}
