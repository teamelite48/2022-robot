package frc.robot.config.pathfollowing;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class PathFollowingConfig {

  // TODO: run system id tool and get real values

  public static final double ksVolts = 0.31107; //0.22;
  public static final double kvVoltSecondsPerMeter = 2.0719; //1.98;
  public static final double kaVoltSecondsSquaredPerMeter = 0.2; //0.2;
  public static final double kPDriveVel = 4.6579; //8.5;

  public static final double trackWidthInMeters = 0.93789; //0.7112;
  public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(trackWidthInMeters);

  public static final double kMaxSpeedMetersPerSecond = 2.875;
  public static final double kMaxAccelerationMetersPerSecondSquared = 1.75;

  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;
 

}
