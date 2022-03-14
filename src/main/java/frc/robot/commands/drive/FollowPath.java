// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.pathfollowing.RamseteCommandFactory;
import frc.robot.RobotContainer;
import frc.robot.pathfollowing.PathType;

public class FollowPath extends SequentialCommandGroup {

  public FollowPath(PathType pathType) {
    RamseteCommandFactory ramseteCommandFactory = new RamseteCommandFactory(RobotContainer.driveSubsystem);

    addCommands(
      ramseteCommandFactory.createCommand(pathType)
    );
  }
}
