// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.turret.EnableAutoAim;
import frc.robot.commands.turret.MoveTurretToDegrees;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShootNear extends SequentialCommandGroup {

  public ShootNear(ShooterSubsystem shooterSubsystem, TurretSubsystem turretSubsystem) {

    addCommands(
      new MoveTurretToDegrees(180, turretSubsystem),
      new EnableAutoAim(turretSubsystem),
      new InstantCommand(shooterSubsystem::setLowSpeed, shooterSubsystem)
    );
  }
}
