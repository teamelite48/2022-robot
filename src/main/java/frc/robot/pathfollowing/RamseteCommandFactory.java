// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pathfollowing;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.config.subsystems.DriveConfig;
import frc.robot.subsystems.DriveSubsystem;


public class RamseteCommandFactory {

  private DriveSubsystem driveSubsystem;

  public RamseteCommandFactory(
    DriveSubsystem driveSubsystem
  ) {
    this.driveSubsystem = driveSubsystem;
  }

  public Command createCommand(PathType pathType) {

    RamseteCommand ramseteCommand = new RamseteCommand(
      TrajectoryFactory.getTrajectory(pathType),
      driveSubsystem::getPose,
      new RamseteController(DriveConfig.kRamseteB, DriveConfig.kRamseteZeta),
      new SimpleMotorFeedforward(
        DriveConfig.ksVolts,
        DriveConfig.kvVoltSecondsPerMeter,
        DriveConfig.kaVoltSecondsSquaredPerMeter
      ),
      DriveConfig.kDriveKinematics,
      driveSubsystem::getWheelSpeeds,
      new PIDController(DriveConfig.kPDriveVel, 0, 0),
      new PIDController(DriveConfig.kPDriveVel, 0, 0),
      driveSubsystem::tankDriveVolts,
      driveSubsystem
    );

    return ramseteCommand.andThen(() -> driveSubsystem.tankDriveVolts(0, 0));
  }
}
