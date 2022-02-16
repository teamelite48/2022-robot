// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.config.roborio.JoystickPort;
import frc.robot.pathfollowing.TrajectoryType;
import frc.robot.pathfollowing.RamseteCommandFactory;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {

    private final Joystick leftPilotJoystick = new Joystick(JoystickPort.LeftPilotJoystick);
    private final Joystick rightPilotJoystick = new Joystick(JoystickPort.RightPilotJoystick);


    private final DriveSubsystem driveSubsystem = new DriveSubsystem();

    private final RamseteCommandFactory ramseteCommandFactory = new RamseteCommandFactory(driveSubsystem);

    private final Command tankDrive = new RunCommand(() -> driveSubsystem.tankDrive(-leftPilotJoystick.getY(), -rightPilotJoystick.getY()), driveSubsystem);

    public RobotContainer() {
        driveSubsystem.setDefaultCommand(tankDrive);

        configureButtonBindings();
    }

    private void configureButtonBindings() {}

    public Command getAutonomousCommand() {
        return ramseteCommandFactory.createCommand(TrajectoryType.Test);
    }
}
