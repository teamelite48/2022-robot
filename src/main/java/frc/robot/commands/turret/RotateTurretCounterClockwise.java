// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurretSubsystem;

public class RotateTurretCounterClockwise extends CommandBase {

  TurretSubsystem turretSubsystem = RobotContainer.turretSubsystem;

  public RotateTurretCounterClockwise() {
    addRequirements(turretSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    turretSubsystem.rotateCounterClockwise();
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
