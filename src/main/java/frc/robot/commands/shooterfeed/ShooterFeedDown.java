// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterfeed;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterFeedSubsystem;


public class ShooterFeedDown extends CommandBase {

  ShooterFeedSubsystem shooterFeedSubsystem = RobotContainer.shooterFeedSubsystem;

  public ShooterFeedDown() {
    addRequirements(shooterFeedSubsystem);
  }

  @Override
  public void initialize() {
    shooterFeedSubsystem.down();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    shooterFeedSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
