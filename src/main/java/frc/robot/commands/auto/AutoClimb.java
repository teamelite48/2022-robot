// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.climber.FullyExtendArms;
import frc.robot.commands.climber.FullyRetractArms;
import frc.robot.commands.climber.TiltArmsDown;
import frc.robot.commands.climber.TiltArmsUp;
import frc.robot.subsystems.ClimberSubsystem;


public class AutoClimb extends SequentialCommandGroup {
  
  public AutoClimb(ClimberSubsystem climberSubsystem)
  {
    addCommands(
      new TiltArmsUp(climberSubsystem),
      new FullyExtendArms(climberSubsystem),
      new WaitCommand(1),
      new FullyRetractArms(climberSubsystem),
      new WaitCommand(1),
      new TiltArmsDown(climberSubsystem),
      new FullyExtendArms(climberSubsystem),
      new TiltArmsUp(climberSubsystem),
      new WaitCommand(1),
      new FullyRetractArms(climberSubsystem),
      new WaitCommand(1),
      new TiltArmsDown(climberSubsystem),
      new FullyExtendArms(climberSubsystem),
      new TiltArmsUp(climberSubsystem),
      new WaitCommand(1),
      new FullyRetractArms(climberSubsystem)
    );
  }
}
