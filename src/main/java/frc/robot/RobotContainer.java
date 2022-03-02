// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.auto.FourBallAuto;
import frc.robot.commands.auto.TestAuto;
import frc.robot.commands.auto.TwoBallAuto;
import frc.robot.commands.climber.ExtendLeftArm;
import frc.robot.commands.climber.ExtendRightArm;
import frc.robot.commands.climber.RetractLeftArm;
import frc.robot.commands.climber.RetractRightArm;
import frc.robot.commands.climber.StopLeftArm;
import frc.robot.commands.climber.StopRightArm;
import frc.robot.commands.climber.ToggleClimberEnabled;
import frc.robot.commands.climber.ToggleLeftArmPosition;
import frc.robot.commands.climber.ToggleRightArmPosition;
import frc.robot.commands.drive.ShiftHighGear;
import frc.robot.commands.drive.ShiftLowGear;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.intake.Outtake;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.shooter.ShootFar;
import frc.robot.commands.shooter.ShootMedium;
import frc.robot.commands.shooter.ShootNear;
import frc.robot.commands.shooter.ToggleShooter;
import frc.robot.commands.shooterfeed.ShooterFeedDown;
import frc.robot.commands.shooterfeed.ShooterFeedStop;
import frc.robot.commands.shooterfeed.ShooterFeedUp;
import frc.robot.commands.sorter.SorterIn;
import frc.robot.commands.sorter.SorterOut;
import frc.robot.commands.sorter.SorterStop;
import frc.robot.commands.turret.RotateTurretClockwise;
import frc.robot.commands.turret.RotateTurretCounterClockwise;
import frc.robot.commands.turret.StopTurret;
import frc.robot.config.roborio.JoystickPort;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterFeedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SorterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
    private final SorterSubsystem sorterSubsystem = new SorterSubsystem();
    private final TurretSubsystem turretSubsystem = new TurretSubsystem();

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {

        driveSubsystem.setDefaultCommand(
            new RunCommand(() -> driveSubsystem.tankDrive(-leftPilotJoystick.getY(), -rightPilotJoystick.getY()), driveSubsystem)
        );

        configurePilotButtonBindings();
        configureCopilotButtonBindings();

        autoChooser.setDefaultOption("Four Ball", new FourBallAuto(driveSubsystem, intakeSubsystem, sorterSubsystem, shooterSubsystem, shooterFeedSubsystem));
        autoChooser.addOption("Two Ball", new TwoBallAuto(driveSubsystem, intakeSubsystem, sorterSubsystem, shooterSubsystem, shooterFeedSubsystem));
        autoChooser.addOption("Test", new TestAuto(driveSubsystem));

        SmartDashboard.putData(autoChooser);
    }


    private void configurePilotButtonBindings() {

        JoystickButton intakeButton = new JoystickButton(leftPilotJoystick, 6);
        JoystickButton outtakeButton = new JoystickButton(rightPilotJoystick, 11);
        JoystickButton retractIntakeButton = new JoystickButton(rightPilotJoystick, 10);

        JoystickButton shiftLowGearButton = new JoystickButton(leftPilotJoystick, 4);
        JoystickButton shiftHighGearButton = new JoystickButton(leftPilotJoystick, 5);

        intakeButton
            .whenPressed(new Intake(intakeSubsystem))
            .whenReleased(new StopIntake(intakeSubsystem));

        outtakeButton
            .whileHeld(new Outtake(intakeSubsystem))
            .whenInactive(new StopIntake(intakeSubsystem));

        retractIntakeButton
            .whenPressed(new RetractIntake(intakeSubsystem));

        shiftLowGearButton
            .whenPressed(new ShiftLowGear(driveSubsystem));

        shiftHighGearButton
            .whenPressed(new ShiftHighGear(driveSubsystem));

    }

    private void configureCopilotButtonBindings() {

        JoystickButton toggleShooterButton = new JoystickButton(copilotGamepad, 2);

        JoystickButton shootNearButton = new JoystickButton(copilotGamepad, 1);
        JoystickButton shootMediumButton = new JoystickButton(copilotGamepad, 3);
        JoystickButton shootFarButton = new JoystickButton(copilotGamepad, 4);

        JoystickButton enableClimberButton1 = new JoystickButton(copilotGamepad, 7);
        JoystickButton enableClimberButton2 = new JoystickButton(copilotGamepad, 8);

        JoystickButton toggleLeftArmPositionButton = new JoystickButton(copilotGamepad, 9);
        JoystickButton toggleRightArmPositionButton = new JoystickButton(copilotGamepad, 10);

        Trigger extendLeftArmTrigger = new Trigger(() -> copilotGamepad.getLeftY() < -0.5);
        Trigger retractLeftArmTrigger = new Trigger(() -> copilotGamepad.getLeftY() > 0.5);

        Trigger extendRightArmTrigger = new Trigger(() -> copilotGamepad.getRightY() < -0.5);
        Trigger retractRightArmTrigger = new Trigger(() -> copilotGamepad.getRightY() > 0.5);

        JoystickButton shooterFeedUpButton = new JoystickButton(copilotGamepad, 6);
        Trigger shooterFeedDownTrigger = new Trigger(() -> copilotGamepad.getRawAxis(3) > 0.5);

        JoystickButton sorterInButton = new JoystickButton(copilotGamepad, 5);
        Trigger sorterOutTrigger = new Trigger(() -> copilotGamepad.getRawAxis(2) > 0.5);

        Trigger rotateTurretClockwiseTrigger = new Trigger(() -> copilotGamepad.getPOV() == 90);
        Trigger rotateTurretCounterClockwiseTrigger = new Trigger(() -> copilotGamepad.getPOV() == 270);

        toggleShooterButton
            .whenPressed(new ToggleShooter(shooterSubsystem));

        shootNearButton
            .whenPressed(new ShootNear(shooterSubsystem));

        shootMediumButton
            .whenPressed(new ShootMedium(shooterSubsystem));

        shootFarButton
            .whenPressed(new ShootFar(shooterSubsystem));

        enableClimberButton1.and(enableClimberButton2)
            .whenActive(new ToggleClimberEnabled(climberSubsystem));

        toggleLeftArmPositionButton
            .whenPressed(new ToggleLeftArmPosition(climberSubsystem));

        toggleRightArmPositionButton
            .whenPressed(new ToggleRightArmPosition(climberSubsystem));

        extendLeftArmTrigger
            .whenActive(new ExtendLeftArm(climberSubsystem))
            .whenInactive(new StopLeftArm(climberSubsystem));

        retractLeftArmTrigger
            .whenActive(new RetractLeftArm(climberSubsystem))
            .whenInactive(new StopLeftArm(climberSubsystem));

        extendRightArmTrigger
            .whenActive(new ExtendRightArm(climberSubsystem))
            .whenInactive(new StopRightArm(climberSubsystem));

        retractRightArmTrigger
            .whenActive(new RetractRightArm(climberSubsystem))
            .whenInactive(new StopRightArm(climberSubsystem));

        shooterFeedUpButton
            .whenPressed(new ShooterFeedUp(shooterFeedSubsystem))
            .whenReleased(new ShooterFeedStop(shooterFeedSubsystem));

        shooterFeedDownTrigger
            .whenActive(new ShooterFeedDown(shooterFeedSubsystem))
            .whenInactive(new ShooterFeedStop(shooterFeedSubsystem));

        sorterInButton
            .whenPressed(new SorterIn(sorterSubsystem))
            .whenReleased(new SorterStop(sorterSubsystem));

        sorterOutTrigger
            .whenActive(new SorterOut(sorterSubsystem))
            .whenInactive(new SorterStop(sorterSubsystem));

        rotateTurretClockwiseTrigger
            .whileActiveContinuous(new RotateTurretClockwise(turretSubsystem))
            .whenInactive(new StopTurret(turretSubsystem));

        rotateTurretCounterClockwiseTrigger
            .whileActiveContinuous(new RotateTurretCounterClockwise(turretSubsystem))
            .whenInactive(new StopTurret(turretSubsystem));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
