// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;


public class ShooterOff extends InstantCommand {

  private ShooterSubsystem shooterSubsystem;

  public ShooterOff(ShooterSubsystem shooterSubsystem) {
    addRequirements(shooterSubsystem);

    this.shooterSubsystem = shooterSubsystem;
  }

  @Override
  public void initialize() {
    shooterSubsystem.shooterOff();
  }
}
