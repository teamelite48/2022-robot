package frc.robot.pathfollowing;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.config.pathfollowing.PathFollowingConfig;

public final class TrajectoryFactory {

  private static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
    new SimpleMotorFeedforward(
      PathFollowingConfig.ksVolts,
      PathFollowingConfig.kvVoltSecondsPerMeter,
      PathFollowingConfig.kaVoltSecondsSquaredPerMeter),
      PathFollowingConfig.kDriveKinematics,
      10
    );

  private static final TrajectoryConfig config = new TrajectoryConfig(
    PathFollowingConfig.kMaxSpeedMetersPerSecond,
    PathFollowingConfig.kMaxAccelerationMetersPerSecondSquared
  )
  .setKinematics(PathFollowingConfig.kDriveKinematics)
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
    new Pose2d(0, 0, new Rotation2d(0)),
    List.of(),
    new Pose2d(-1, 0, new Rotation2d(0)),
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

  public static Trajectory createTrajectory(TrajectoryType trajectoryType) {
    switch (trajectoryType) {

      case Test: {
        return testTrajectory;
      }

      case BackOffLine: {
        return backOffLine;
      }

      case GetReadyToShoot: {
        return getReadyToShoot;
      }
    }

    return null;
  }
}
