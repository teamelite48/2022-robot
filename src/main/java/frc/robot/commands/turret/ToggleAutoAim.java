// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.TurretSubsystem;


public class ToggleAutoAim extends InstantCommand {

  TurretSubsystem turretSubsystem;

  public ToggleAutoAim(TurretSubsystem turretSubsystem) {
    addRequirements(turretSubsystem);

    this.turretSubsystem = turretSubsystem;
  }

  @Override
  public void initialize() {
    turretSubsystem.toggleAutoAim();
  }
}
