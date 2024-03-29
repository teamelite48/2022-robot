// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterfeed;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterFeedSubsystem;

public class ShooterFeedStop extends InstantCommand {

  ShooterFeedSubsystem shooterFeedSubsystem = RobotContainer.shooterFeedSubsystem;

  public ShooterFeedStop() {
    addRequirements(shooterFeedSubsystem);
  }

  @Override
  public void initialize() {
    shooterFeedSubsystem.stop();
  }
}
