package frc.robot.pathfollowing;

import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.config.sysid.SysIdConfig;

public final class TrajectoryFactory {

  private static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
    new SimpleMotorFeedforward(
      SysIdConfig.ksVolts,
      SysIdConfig.kvVoltSecondsPerMeter,
      SysIdConfig.kaVoltSecondsSquaredPerMeter),
      SysIdConfig.kDriveKinematics,
      10
    );

  private static final TrajectoryConfig config = new TrajectoryConfig(
    SysIdConfig.kMaxSpeedMetersPerSecond,
    SysIdConfig.kMaxAccelerationMetersPerSecondSquared
  )
  .setKinematics(SysIdConfig.kDriveKinematics)
  .addConstraint(autoVoltageConstraint);

  private static final Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d(0)),
    List.of(
      new Translation2d(1, 1),
      new Translation2d(2, -1)
    ),
    new Pose2d(3, 0, new Rotation2d(0)),
    config
  );

  private static final Trajectory backOffLine = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))),
    List.of(new Translation2d(2, 0)),
    new Pose2d(3, 3, new Rotation2d(Math.toRadians(0))),
    config
  );

  private static final Trajectory getReadyToShoot = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d(0)),
    List.of(
      new Translation2d(5.5, 2)
    ),
    new Pose2d(7.4, 2, new Rotation2d(Math.toRadians(70))),
    config
  );

  private static final Trajectory getBackUpTrajectory() {

    String trajectoryJSON = "paths/BackUp.wpilib.json";
    Trajectory trajectory = null;

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }
    catch ( Exception ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    return trajectory;
  }

  private static final Trajectory getPullForwardTrajectory() {

    String trajectoryJSON = "paths/PullForward.wpilib.json";
    Trajectory trajectory = null;

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }
    catch ( Exception ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    return trajectory;
  }

  private static final Trajectory getSeriouslyBackUpTrajectory() {

    String trajectoryJSON = "paths/SeriouslyBackUp.wpilib.json";
    Trajectory trajectory = null;

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }
    catch ( Exception ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    return trajectory;
  }

  private static final Trajectory getSeriouslyPullForwardTrajectory() {

    String trajectoryJSON = "paths/SeriouslyPullForward.wpilib.json";
    Trajectory trajectory = null;

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }
    catch ( Exception ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    return trajectory;
  }

  public static Trajectory getTrajectory(TrajectoryType pathType) {
    switch (pathType) {

      case Test: {
        return testTrajectory;
      }

      case BackOffLine: {
        return backOffLine;
      }

      case GetReadyToShoot: {
        return getReadyToShoot;
      }

      case BackUp: {
        return getBackUpTrajectory();
      }

      case PullForward: {
        return getPullForwardTrajectory();
      }

      case SeriouslyBackUp: {
        return getSeriouslyBackUpTrajectory();
      }

      case SeriouslyPullForward: {
        return getSeriouslyPullForwardTrajectory();
      }
    }

    return null;
  }
}
