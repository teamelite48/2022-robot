// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntake extends InstantCommand {

  IntakeSubsystem intakeSubsystem = RobotContainer.intakeSubsystem;

  public AutoIntake() {
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.intake();
  }
}
