// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.ResetOdometry;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.shooter.ShootMedium;
import frc.robot.commands.shooter.ShooterOff;
import frc.robot.commands.shooter.ShooterOn;
import frc.robot.commands.shooterfeed.ShooterFeedStop;
import frc.robot.commands.shooterfeed.ShooterFeedUp;
import frc.robot.commands.sorter.SorterIn;
import frc.robot.commands.sorter.SorterStop;
import frc.robot.pathfollowing.RamseteCommandFactory;
import frc.robot.pathfollowing.TrajectoryType;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterFeedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SorterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class FourBallStraightAuto extends SequentialCommandGroup {

  public FourBallStraightAuto(
    DriveSubsystem driveSubsystem,
    IntakeSubsystem intakeSubsystem,
    SorterSubsystem sorterSubsystem,
    ShooterSubsystem shooterSubsystem,
    ShooterFeedSubsystem shooterFeedSubsystem,
    TurretSubsystem turretSubsystem
  ) {

    RamseteCommandFactory ramseteCommandFactory = new RamseteCommandFactory(driveSubsystem);


    addCommands(
      new ResetOdometry(9.9, 5.6, 160, driveSubsystem),                 //degrees need adjusted probably
      new InstantCommand(intakeSubsystem::deploy, intakeSubsystem),
      new WaitCommand(0.2),
      new InstantCommand(intakeSubsystem::intake, intakeSubsystem),
      new WaitCommand(0.3),
      new SorterIn(sorterSubsystem),
      ramseteCommandFactory.createCommand(TrajectoryType.FourBallStraight1),
      new RetractIntake(intakeSubsystem),
      //ramseteCommandFactory.createCommand(TrajectoryType.FourBallStraight2),
      new ShooterOn(shooterSubsystem),
      new ShootMedium(shooterSubsystem, turretSubsystem),
      new WaitCommand(0.5),
      new ShooterFeedUp(shooterFeedSubsystem),
      new WaitCommand(2.5),
      new ShooterFeedStop(shooterFeedSubsystem),
      //new ShooterOff(shooterSubsystem),
      new InstantCommand(intakeSubsystem::deploy, intakeSubsystem),
      new WaitCommand(0.2),
      new InstantCommand(intakeSubsystem::intake, intakeSubsystem),
      ramseteCommandFactory.createCommand(TrajectoryType.FourBallStraight3),
      new WaitCommand(1.5),
      //new ShooterOn(shooterSubsystem),
      ramseteCommandFactory.createCommand(TrajectoryType.FourBallStraight4),
      new RetractIntake(intakeSubsystem),
      new ShootMedium(shooterSubsystem, turretSubsystem),
      new ShooterFeedUp(shooterFeedSubsystem),
      new WaitCommand(2.5),
      new ShooterFeedStop(shooterFeedSubsystem),
      new SorterStop(sorterSubsystem),
      new ShooterOff(shooterSubsystem)
    );
  }
}