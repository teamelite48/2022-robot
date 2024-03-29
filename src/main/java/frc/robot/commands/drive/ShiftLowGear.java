// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;


public class ShiftLowGear extends InstantCommand {

  DriveSubsystem driveSubsystem  = RobotContainer.driveSubsystem;

  public ShiftLowGear() {}

  @Override
  public void initialize() {
    driveSubsystem.shiftLowGear();
  }
}
