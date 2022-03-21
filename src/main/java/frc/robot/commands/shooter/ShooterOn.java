// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;


public class ShooterOn extends InstantCommand {

  private ShooterSubsystem shooterSubsystem = RobotContainer.shooterSubsystem;

  public ShooterOn() {
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.shooterOn();
    
    RobotContainer.controller.setRumble(RumbleType.kLeftRumble, .75);
    RobotContainer.controller.setRumble(RumbleType.kRightRumble, .75);
  }
}
