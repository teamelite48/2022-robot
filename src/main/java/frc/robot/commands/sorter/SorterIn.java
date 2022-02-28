// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sorter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SorterSubsystem;


public class SorterIn extends InstantCommand {

  SorterSubsystem sorterSubsystem;

  public SorterIn(SorterSubsystem sorterSubsystem) {

    addRequirements(sorterSubsystem);

    this.sorterSubsystem = sorterSubsystem;
  }

  @Override
  public void initialize() {
    sorterSubsystem.in();
  }
}