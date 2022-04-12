// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.ShiftHighGear;
import frc.robot.commands.turret.EnableTurret;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.TurretSubsystem;


public class DisableClimber extends SequentialCommandGroup {

  ClimberSubsystem climberSubsystem = RobotContainer.climberSubsystem;
  TurretSubsystem turretSubsystem = RobotContainer.turretSubsystem;

  public DisableClimber() {
    addCommands(
      new InstantCommand(climberSubsystem::disableClimber),
      new ShiftHighGear(),
      new EnableTurret()
    );
  }
}
