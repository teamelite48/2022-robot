// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.FollowPath;
import frc.robot.commands.drive.ResetOdometry;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.shooter.ShooterOff;
import frc.robot.commands.shooter.ShooterOn;
import frc.robot.commands.shooterfeed.ShooterFeedStop;
import frc.robot.commands.shooterfeed.ShooterFeedUp;
import frc.robot.commands.sorter.SorterIn;
import frc.robot.pathfollowing.PathType;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterFeedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SorterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class FourBallAuto extends SequentialCommandGroup {

  public FourBallAuto(
    DriveSubsystem driveSubsystem,
    IntakeSubsystem intakeSubsystem,
    SorterSubsystem sorterSubsystem,
    ShooterSubsystem shooterSubsystem,
    ShooterFeedSubsystem shooterFeedSubsystem,
    TurretSubsystem turretSubsystem
  ) {

    addCommands(
      new ResetOdometry(8.7, 6.4, 88, driveSubsystem),
      new Intake(intakeSubsystem),
      new SorterIn(sorterSubsystem),
      new FollowPath(PathType.FourBall1),
      new ShooterOn(shooterSubsystem),
      new FollowPath(PathType.FourBall2),
      new ShooterFeedUp(shooterFeedSubsystem),
      new WaitCommand(1),
      new ShooterFeedStop(shooterFeedSubsystem),
      new ShooterOff(shooterSubsystem),
      new FollowPath(PathType.FourBall3),
      new ShooterOn(shooterSubsystem),
      new FollowPath(PathType.FourBall4),
      new ShooterFeedUp(shooterFeedSubsystem),
      new WaitCommand(1),
      new ShooterFeedStop(shooterFeedSubsystem),
      new ShooterOff(shooterSubsystem)
    );
  }
}
