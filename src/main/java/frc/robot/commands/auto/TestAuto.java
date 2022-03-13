// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.FollowPath;
import frc.robot.commands.drive.ResetOdometry;
import frc.robot.pathfollowing.PathType;
import frc.robot.subsystems.DriveSubsystem;


public class TestAuto extends SequentialCommandGroup {

  public TestAuto(DriveSubsystem driveSubsystem) {

    addCommands(
      new ResetOdometry(0, 0, 0, driveSubsystem),
      new FollowPath(PathType.Test)
    );
  }
}
