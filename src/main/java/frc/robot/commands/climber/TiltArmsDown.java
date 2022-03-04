// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimberSubsystem;


public class TiltArmsDown extends InstantCommand {

  ClimberSubsystem climberSubsystem;

  public TiltArmsDown(ClimberSubsystem climberSubsystem) {
    addRequirements(climberSubsystem);

    this.climberSubsystem = climberSubsystem;
  }

  @Override
  public void initialize() {
    climberSubsystem.tiltArmsDown();
  }
}