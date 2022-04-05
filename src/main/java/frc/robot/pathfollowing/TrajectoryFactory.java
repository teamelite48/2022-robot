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

  public static Trajectory getTrajectory(PathType pathType) {
    switch (pathType) {

      case Test: {
        return testTrajectory;
      }

      case FourBall1: {
        return getTrajectoryFromFile("FourBall1.wpilib.json");
      }

      case FourBall2: {
        return getTrajectoryFromFile("FourBall2.wpilib.json");
      }

      case FourBall3: {
        return getTrajectoryFromFile("FourBall3.wpilib.json");
      }

      case FourBall4: {
        return getTrajectoryFromFile("FourBall4.wpilib.json");
      }

      case TwoBall1: {
        return getTrajectoryFromFile("TwoBall1.wpilib.json");
      }

      case TwoBall2:{
        return getTrajectoryFromFile("TwoBall2.wpilib.json");
      }

      case TwoBallShort1:{
        return getTrajectoryFromFile("TwoBallShort1.wpilib.json");
      }

      case BackOffLine:{
        return getTrajectoryFromFile("BackOffLine.wpilib.json");
      }

      case FourBallStraight1: {
        return getTrajectoryFromFile("FourBallStraight1.wpilib.json");
      }

      case FourBallStraight2: {
        return getTrajectoryFromFile("FourBallStraight2.wpilib.json");
      }

      case FourBallStraight3: {
        return getTrajectoryFromFile("FourBallStraight3.wpilib.json");
      }

      case FourBallStraight4: {
        return getTrajectoryFromFile("FourBallStraight4.wpilib.json");
      }

      case FourBallStraight5: {
        return getTrajectoryFromFile("FourBallStraight5.wpilib.json");
      }

    }



    return null;
  }
}
