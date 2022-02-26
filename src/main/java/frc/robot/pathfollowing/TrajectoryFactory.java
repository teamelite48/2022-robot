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

  private static final Trajectory getTrajectoryFromFile(String filename) {

    String trajectoryJson = "paths/" + filename;

    Trajectory trajectory = null;

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJson);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }
    catch ( Exception ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJson, ex.getStackTrace());
    }

    return trajectory;
  }

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

  public static Trajectory getTrajectory(TrajectoryType pathType) {
    switch (pathType) {

      case Test: {
        return testTrajectory;
      }

      case BackUp: {
        return getTrajectoryFromFile("BackUp.wpilib.json");
      }

      case PullForward: {
        return getTrajectoryFromFile("PullForward.wpilib.json");
      }

      case SeriouslyBackUp: {
        return getTrajectoryFromFile("SeriouslyBackUp.wpilib.json");
      }

      case SeriouslyPullForward: {
        return getTrajectoryFromFile("SeriouslyPullForward.wpilib.json");
      }
    }

    return null;
  }
}
