// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.CoolDownTimer;

public class BackOffLineDeadReckoning extends CommandBase {

  DriveSubsystem driveSubsystem;
  CoolDownTimer coolDown = new CoolDownTimer(3000);

  public BackOffLineDeadReckoning(DriveSubsystem driveSubsystem) {
    addRequirements(driveSubsystem);

    this.driveSubsystem = driveSubsystem;
  }

  @Override
  public void initialize() {
    coolDown.start();
  }

  @Override
  public void execute() {
    driveSubsystem.tankDrive(-.3, -.3, 1);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.tankDrive(0, 0, 1);
  }

  @Override
  public boolean isFinished() {
    return coolDown.isCool();
  }
}
