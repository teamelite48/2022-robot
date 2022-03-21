// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.FollowPath;
import frc.robot.commands.drive.ResetOdometry;
import frc.robot.commands.intake.AutoIntake;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.shooter.ShootMedium;
import frc.robot.commands.shooter.ShooterOff;
import frc.robot.commands.shooterfeed.ShooterFeedStop;
import frc.robot.commands.shooterfeed.AutoShooterFeedUp;
import frc.robot.commands.sorter.SorterIn;
import frc.robot.commands.sorter.SorterStop;
import frc.robot.pathfollowing.PathType;
import frc.robot.subsystems.IntakeSubsystem;


public class FourBallStraightAuto extends SequentialCommandGroup {

  IntakeSubsystem intakeSubsystem = RobotContainer.intakeSubsystem;

  public FourBallStraightAuto() {
    addCommands(
      new ResetOdometry(9.9, 5.6, 160),
      new AutoIntake(),
      new WaitCommand(.75),
      new SorterIn(),
      new ShootMedium(),
      new FollowPath(PathType.FourBallStraight1),
      new RetractIntake(),
      new WaitCommand(.3),
      new AutoShooterFeedUp(),
      new WaitCommand(2.5),
      new ShooterFeedStop(),
      new ShooterOff(),
      new AutoIntake(),
      new FollowPath(PathType.FourBallStraight3),
      new WaitCommand(1.3),
      new RetractIntake(),
      new FollowPath(PathType.FourBallStraight4),
      new ShootMedium(),
      new WaitCommand(.3),
      new AutoShooterFeedUp(),
      new WaitCommand(3),
      new ShooterFeedStop(),
      new SorterStop(),
      new ShooterOff()
    );
  }
}