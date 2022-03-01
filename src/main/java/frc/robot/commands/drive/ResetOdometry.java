// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class ResetOdometry extends InstantCommand {

  DriveSubsystem driveSubsystem;
  double x;
  double y;
  double degrees;


  public ResetOdometry(double x, double y, double degrees, DriveSubsystem driveSubsystem) {

    addRequirements(driveSubsystem);

    this.driveSubsystem = driveSubsystem;
    this.x = x;
    this.y = y;
    this.degrees = degrees;
  }

  @Override
  public void initialize() {
    driveSubsystem.resetOdometry(x, y, degrees);
  }
}
