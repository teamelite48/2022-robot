// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.config.roborio.JoystickPort;
import frc.robot.pathfollowing.TrajectoryType;
import frc.robot.pathfollowing.RamseteCommandFactory;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterFeedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

    private final Joystick leftPilotJoystick = new Joystick(JoystickPort.LeftPilotJoystick);
    private final Joystick rightPilotJoystick = new Joystick(JoystickPort.RightPilotJoystick);
    private final XboxController copilotGamepad = new XboxController(JoystickPort.CopilotGamepad);

    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    private final ShooterFeedSubsystem shooterFeedSubsystem = new ShooterFeedSubsystem();

    private final RamseteCommandFactory ramseteCommandFactory = new RamseteCommandFactory(driveSubsystem);

    private final Command tankDrive = new RunCommand(() -> driveSubsystem.tankDrive(-leftPilotJoystick.getY(), -rightPilotJoystick.getY()), driveSubsystem);
    private final Command intake = new InstantCommand(() -> intakeSubsystem.intake(), intakeSubsystem);
    private final Command outtake = new RunCommand(() -> intakeSubsystem.outtake(), intakeSubsystem);
    private final Command stopIntake = new InstantCommand(() -> intakeSubsystem.stop(), intakeSubsystem);
    private final Command retractIntake = new InstantCommand(() -> intakeSubsystem.retract(), intakeSubsystem);

    private final Command shoot = new RunCommand(() -> shooterSubsystem.shoot(), shooterSubsystem);

    private final Command toggleClimberEnabled = new InstantCommand (()-> climberSubsystem.toggleClimberEnabled(), climberSubsystem);

    private final Command toggleLeftArmPosition = new InstantCommand(() -> climberSubsystem.toggleLeftArmPosition(), climberSubsystem);
    private final Command toggleRightArmPosition = new InstantCommand(() -> climberSubsystem.toggleRightArmPosition(), climberSubsystem);

    private final Command extendLeftArm = new InstantCommand(() -> climberSubsystem.extendLeftArm(), climberSubsystem);
    private final Command retractLeftArm = new InstantCommand(() -> climberSubsystem.retractLeftArm(), climberSubsystem);
    private final Command stopLeftArm = new InstantCommand(() -> climberSubsystem.stopLeftArm(), climberSubsystem);

    private final Command extendRightArm = new InstantCommand(() -> climberSubsystem.extendRightArm(), climberSubsystem);
    private final Command retractRightArm = new InstantCommand(() -> climberSubsystem.retractRightArm(), climberSubsystem);
    private final Command stopRightArm = new InstantCommand(() -> climberSubsystem.stopRightArm(), climberSubsystem);


    private final Command shooterFeedUp = new InstantCommand(() -> shooterFeedSubsystem.up(), shooterSubsystem);
    private final Command shooterFeedDown = new InstantCommand(() -> shooterFeedSubsystem.down(), shooterSubsystem);
    private final Command shooterFeedStop = new InstantCommand(() -> shooterFeedSubsystem.stop(), shooterSubsystem);

    private final Command driveToGoal = ramseteCommandFactory.createCommand(TrajectoryType.GetReadyToShoot);

    private final Command pickUpCargoAndShoot = new SequentialCommandGroup(
        intake,
        driveToGoal,
        retractIntake,
        shooterFeedUp,
        shoot
    );

    public RobotContainer() {
        driveSubsystem.setDefaultCommand(tankDrive);

        configurePilotButtonBindings();
        configureCopilotButtonBindings();
    }

    private void configurePilotButtonBindings() {

        JoystickButton intakeButton = new JoystickButton(leftPilotJoystick, 6);
        JoystickButton outtakeButton = new JoystickButton(rightPilotJoystick, 11);
        JoystickButton retractIntakeButton = new JoystickButton(rightPilotJoystick, 10);

        intakeButton
            .whenPressed(intake)
            .whenReleased(stopIntake);

        outtakeButton
            .whileHeld(outtake)
            .whenInactive(stopIntake);

        retractIntakeButton
            .whenPressed(retractIntake);
    }

    private void configureCopilotButtonBindings() {

        JoystickButton shootButton = new JoystickButton(copilotGamepad, 2);

        JoystickButton enableClimberButton1 = new JoystickButton(copilotGamepad, 7);
        JoystickButton enableClimberButton2 = new JoystickButton(copilotGamepad, 8);

        JoystickButton toggleLeftArmPositionButton = new JoystickButton(copilotGamepad, 9);
        JoystickButton toggleRightArmPositionButton = new JoystickButton(copilotGamepad, 10);

        Trigger extendLeftArmTrigger = new Trigger(() -> copilotGamepad.getLeftY() < -0.5);
        Trigger retractLeftArmTrigger = new Trigger(() -> copilotGamepad.getLeftY() > 0.5);

        Trigger extendRightArmTrigger = new Trigger(() -> copilotGamepad.getRightY() < -0.5);
        Trigger retractRightArmTrigger = new Trigger(() -> copilotGamepad.getRightY() > 0.5);

        JoystickButton shooterFeedUpButton = new JoystickButton(copilotGamepad, 5);
        JoystickButton shooterFeedDownButton = new JoystickButton(copilotGamepad, 6);

        shootButton
            .whileHeld(shoot);

        enableClimberButton1.and(enableClimberButton2)
            .whenActive(toggleClimberEnabled);

        toggleLeftArmPositionButton
            .whenPressed(toggleLeftArmPosition);

        toggleRightArmPositionButton
            .whenPressed(toggleRightArmPosition);

        extendLeftArmTrigger
            .whenActive(extendLeftArm)
            .whenInactive(stopLeftArm);

        retractLeftArmTrigger
            .whenActive(retractLeftArm)
            .whenInactive(stopLeftArm);

        extendRightArmTrigger
            .whenActive(extendRightArm)
            .whenInactive(stopRightArm);

        retractRightArmTrigger
            .whenActive(retractRightArm)
            .whenInactive(stopRightArm);

        shooterFeedUpButton
            .whenPressed(shooterFeedUp)
            .whenReleased(shooterFeedStop);

        shooterFeedDownButton
            .whenPressed(shooterFeedDown)
            .whenReleased(shooterFeedStop);
    }

    public Command getAutonomousCommand() {
        return pickUpCargoAndShoot;
    }
}
