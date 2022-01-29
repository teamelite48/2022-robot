// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.config.JoystickPort;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {

    private final Joystick leftPilotJoystick = new Joystick(JoystickPort.LeftPilotJoystick);
    private final Joystick rightPilotJoystick = new Joystick(JoystickPort.RightPilotJoystick);

    private final DriveSubsystem driveSubsystem = new DriveSubsystem();

    private final Command tankDrive = new RunCommand(() -> driveSubsystem.tankDrive(leftPilotJoystick.getY(), rightPilotJoystick.getY()), driveSubsystem);

    // creates field for simmulation
    private Field2d field = new Field2d();

    public RobotContainer() {
        driveSubsystem.setDefaultCommand(tankDrive);

        configureButtonBindings();

        SmartDashboard.putData("Field", field);
    }

    private void configureButtonBindings() {

    }

}
