package frc.robot.config.pathfollowing;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class PathFollowingConfig {

  // TODO: run system id tool and get real values

  public static final double ksVolts = 6.063; //0.22;
  public static final double kvVoltSecondsPerMeter = 1.2311; //1.98;
  public static final double kaVoltSecondsSquaredPerMeter = -37.614; //0.2;
  public static final double kPDriveVel = 0; //8.5;

  public static final double trackWidthInMeters = 0.7112;
  public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(trackWidthInMeters);

  public static final int kMaxSpeedMetersPerSecond = 1;

  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;

}
