// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.turret.EnableAutoAim;
import frc.robot.commands.turret.MoveTurretToDegrees;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShootMedium extends SequentialCommandGroup {

  public ShootMedium() {

    ShooterSubsystem shooterSubsystem = RobotContainer.shooterSubsystem;

    addCommands(
      new InstantCommand(shooterSubsystem::setMediumSpeed, shooterSubsystem),
      new MoveTurretToDegrees(180),
      new EnableAutoAim()
    );
  }
}
