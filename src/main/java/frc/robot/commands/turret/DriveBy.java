// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class DriveBy extends SequentialCommandGroup {

  public DriveBy(double degrees, TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem) {

    addCommands(
      new MoveTurretToDegrees(degrees, turretSubsystem),
      new InstantCommand(shooterSubsystem::setLowSpeed, shooterSubsystem)
    );
  }
}
