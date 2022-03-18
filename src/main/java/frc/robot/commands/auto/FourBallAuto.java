// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.FollowPath;
import frc.robot.commands.drive.ResetOdometry;
import frc.robot.commands.intake.AutoIntake;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.shooter.ShootMedium;
import frc.robot.commands.shooter.ShooterOff;
import frc.robot.commands.shooterfeed.ShooterFeedStop;
import frc.robot.commands.shooterfeed.AutoShooterFeedUp;
import frc.robot.commands.sorter.SorterIn;
import frc.robot.pathfollowing.PathType;

public class FourBallAuto extends SequentialCommandGroup {

  public FourBallAuto() {
    addCommands(
      new ResetOdometry(8.7, 6.4, 88),
      new AutoIntake(),
      new SorterIn(),
      new FollowPath(PathType.FourBall1),
      new ShootMedium(),
      new FollowPath(PathType.FourBall2),
      new AutoShooterFeedUp(),
      new WaitCommand(1),
      new ShooterFeedStop(),
      new ShooterOff(),
      new FollowPath(PathType.FourBall3),
      new ShootMedium(),
      new FollowPath(PathType.FourBall4),
      new RetractIntake(),
      new AutoShooterFeedUp(),
      new WaitCommand(1),
      new ShooterFeedStop(),
      new ShooterOff()
    );
  }
}
