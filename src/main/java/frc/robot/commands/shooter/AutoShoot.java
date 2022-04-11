// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.turret.AutoAimOn;


public class AutoShoot extends SequentialCommandGroup {

  public AutoShoot() {

    addCommands(
      new AutoAimOn(),
      new TurnRangeBasedShooterRpmOn(),
      new ShooterOn()
    );
  }
}
