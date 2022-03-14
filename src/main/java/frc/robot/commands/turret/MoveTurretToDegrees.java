// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.config.subsystems.TurretConfig;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.CoolDownTimer;

public class MoveTurretToDegrees extends CommandBase {

  TurretSubsystem turretSubsystem = RobotContainer.turretSubsystem;
  CoolDownTimer coolDown = new CoolDownTimer(TurretConfig.moveCoolDown);
  double degrees;

  public MoveTurretToDegrees(double degrees) {
    addRequirements(turretSubsystem);

    this.degrees = degrees;
  }

  @Override
  public void initialize() {
    coolDown.start();
  }

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
    return Math.abs(turretSubsystem.getPositionInDegrees() - degrees) < TurretConfig.moveWithinDegrees
      || coolDown.isCool();
  }
}
