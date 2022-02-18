// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pathfollowing;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.config.pathfollowing.PathFollowingConfig;
import frc.robot.subsystems.DriveSubsystem;


public class RamseteCommandFactory {

  private DriveSubsystem driveSubsystem;

  public RamseteCommandFactory(
    DriveSubsystem _driveSubsystem
  ) {
    driveSubsystem = _driveSubsystem;
  }

  public Command createCommand(TrajectoryType trajectoryType) {

    RamseteCommand ramseteCommand = new RamseteCommand(
      TrajectoryFactory.createTrajectory(trajectoryType),
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
