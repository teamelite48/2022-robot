// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class MoveTurretToDegrees extends CommandBase {

  TurretSubsystem turretSubsystem;
  double degrees;

  public MoveTurretToDegrees(double degrees, TurretSubsystem turretSubsystem) {
    addRequirements(turretSubsystem);

    this.turretSubsystem = turretSubsystem;
    this.degrees = degrees;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    turretSubsystem.disableAutoAim();
    turretSubsystem.moveToDegrees(degrees);
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(turretSubsystem.getPositionInDegrees() - degrees) < 2;
  }
}
