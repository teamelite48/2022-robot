// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.turret.MoveTurretToDegrees;
import frc.robot.config.subsystems.TurretConfig;
import frc.robot.subsystems.ShooterSubsystem;


public class ShooterOff extends SequentialCommandGroup {

  private ShooterSubsystem shooterSubsystem = RobotContainer.shooterSubsystem;

  public ShooterOff() {
    addCommands(
      new InstantCommand(shooterSubsystem::shooterOff, shooterSubsystem),
      new MoveTurretToDegrees(TurretConfig.degreesAtCenter)
    );

    RobotContainer.controller.setRumble(RumbleType.kLeftRumble, 0);
    RobotContainer.controller.setRumble(RumbleType.kRightRumble, 0);
  }
}
