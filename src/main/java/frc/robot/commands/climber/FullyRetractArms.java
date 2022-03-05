// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class FullyRetractArms extends CommandBase {
  
  ClimberSubsystem climberSubsystem;

  public FullyRetractArms(ClimberSubsystem climberSubsystem) {
    addRequirements(climberSubsystem);

    this.climberSubsystem = climberSubsystem;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.climberSubsystem.retractArms();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return this.climberSubsystem.isFullyRetracted();
  }
}
