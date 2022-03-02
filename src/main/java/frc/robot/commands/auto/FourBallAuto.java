// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.ResetOdometry;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.shooter.ShooterOff;
import frc.robot.commands.shooter.ShooterOn;
import frc.robot.commands.shooter.ToggleShooter;
import frc.robot.commands.shooterfeed.ShooterFeedStop;
import frc.robot.commands.shooterfeed.ShooterFeedUp;
import frc.robot.commands.sorter.SorterIn;
import frc.robot.pathfollowing.RamseteCommandFactory;
import frc.robot.pathfollowing.TrajectoryType;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterFeedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SorterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourBallAuto extends SequentialCommandGroup {

  public FourBallAuto(
    DriveSubsystem driveSubsystem,
    IntakeSubsystem intakeSubsystem,
    SorterSubsystem sorterSubsystem,
    ShooterSubsystem shooterSubsystem,
    ShooterFeedSubsystem shooterFeedSubsystem
  ) {

    RamseteCommandFactory ramseteCommandFactory = new RamseteCommandFactory(driveSubsystem);


    addCommands(
      new ResetOdometry(9, 6.5, 88, driveSubsystem),
      new Intake(intakeSubsystem),
      new SorterIn(sorterSubsystem),
      ramseteCommandFactory.createCommand(TrajectoryType.FourBall1),
      new ShooterOn(shooterSubsystem),
      ramseteCommandFactory.createCommand(TrajectoryType.FourBall2),
      new ShooterFeedUp(shooterFeedSubsystem),
      new WaitCommand(1),
      new ShooterFeedStop(shooterFeedSubsystem),
      new ShooterOff(shooterSubsystem),
      ramseteCommandFactory.createCommand(TrajectoryType.FourBall3),
      new ShooterOn(shooterSubsystem),
      ramseteCommandFactory.createCommand(TrajectoryType.FourBall4),
      new ShooterFeedUp(shooterFeedSubsystem),
      new WaitCommand(1),
      new ShooterFeedStop(shooterFeedSubsystem),
      new ShooterOff(shooterSubsystem)
    );
  }
}
