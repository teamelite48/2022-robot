// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem;


public class ExtendArms extends CommandBase {

  ClimberSubsystem climberSubsystem = RobotContainer.climberSubsystem;

  public ExtendArms() {
    addRequirements(climberSubsystem);
  }

  @Override
  public void initialize() {
    climberSubsystem.extendArms();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    climberSubsystem.stopArms();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
