// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pathfollowing;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.config.pathfollowing.PathFollowingConfig;
import frc.robot.subsystems.DriveSubsystem;


public class RamseteFactory {

  private DriveSubsystem driveSubsystem;

  public RamseteFactory(
    DriveSubsystem _driveSubsystem
  ) {
    driveSubsystem = _driveSubsystem;
  }

  public Command simple() {
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
        PathFollowingConfig.ksVolts,
        PathFollowingConfig.kvVoltSecondsPerMeter,
        PathFollowingConfig.kaVoltSecondsSquaredPerMeter),
        PathFollowingConfig.kDriveKinematics,
        10
      );

    TrajectoryConfig config = new TrajectoryConfig(
      PathFollowingConfig.kMaxSpeedMetersPerSecond,
      PathFollowingConfig.kMaxAccelerationMetersPerSecondSquared
    );

    config
      .setKinematics(PathFollowingConfig.kDriveKinematics)
      .addConstraint(autoVoltageConstraint);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        new Translation2d(1, 1),
        new Translation2d(2, -1)
      ),
      new Pose2d(3, 0, new Rotation2d(0)),
      config
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
      exampleTrajectory,
      driveSubsystem::getPose,
      new RamseteController(PathFollowingConfig.kRamseteB, PathFollowingConfig.kRamseteZeta),
      new SimpleMotorFeedforward(
        PathFollowingConfig.ksVolts,
        PathFollowingConfig.kvVoltSecondsPerMeter,
        PathFollowingConfig.kaVoltSecondsSquaredPerMeter
      ),
      PathFollowingConfig.kDriveKinematics,
      driveSubsystem::getWheelSpeeds,
      new PIDController(PathFollowingConfig.kPDriveVel, 0, 0),
      new PIDController(PathFollowingConfig.kPDriveVel, 0, 0),
      driveSubsystem::tankDriveVolts,
      driveSubsystem
    );

    return ramseteCommand.andThen(() -> driveSubsystem.tankDriveVolts(0, 0));
  }
}
