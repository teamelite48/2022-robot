// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.auto.AutoClimb;
import frc.robot.commands.auto.BackOffLineAuto;
import frc.robot.commands.auto.FourBallAuto;
import frc.robot.commands.auto.TestAuto;
import frc.robot.commands.auto.TwoBallAuto;
import frc.robot.commands.climber.ExtendArms;
import frc.robot.commands.climber.RetractArms;
import frc.robot.commands.climber.StopArms;
import frc.robot.commands.climber.ToggleArmPositions;
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
import frc.robot.commands.turret.EnableAutoAim;
import frc.robot.commands.turret.MoveTurretToDegrees;
import frc.robot.commands.turret.DisableAutoAim;
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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
            new RunCommand(() -> driveSubsystem.tankDrive(-leftPilotJoystick.getY(), -rightPilotJoystick.getY(), rightPilotJoystick.getRawAxis(2)), driveSubsystem)
        );

        turretSubsystem.setDefaultCommand(
            new RunCommand(() -> turretSubsystem.manualTurret(copilotGamepad.getLeftX()), turretSubsystem)
        );

        configurePilotButtonBindings();
        configureCopilotButtonBindings();

        autoChooser.setDefaultOption("Do Nothing", new WaitCommand(1));
        autoChooser.addOption("Four Ball", new FourBallAuto(driveSubsystem, intakeSubsystem, sorterSubsystem, shooterSubsystem, shooterFeedSubsystem));
        autoChooser.addOption("Two Ball", new TwoBallAuto(driveSubsystem, intakeSubsystem, sorterSubsystem, shooterSubsystem, shooterFeedSubsystem));
        autoChooser.addOption("Back Off Line", new BackOffLineAuto(driveSubsystem));
        autoChooser.addOption("Test", new TestAuto(driveSubsystem));


        SmartDashboard.putData(autoChooser);
    }


    private void configurePilotButtonBindings() {

        JoystickButton intakeButton = new JoystickButton(leftPilotJoystick, 6);
        JoystickButton outtakeButton = new JoystickButton(rightPilotJoystick, 11);
        JoystickButton retractIntakeButton = new JoystickButton(rightPilotJoystick, 10);

        JoystickButton shiftLowGearButton = new JoystickButton(leftPilotJoystick, 4);
        JoystickButton shiftHighGearButton = new JoystickButton(leftPilotJoystick, 5);

        JoystickButton autoClimbButton = new JoystickButton(rightPilotJoystick, 3);

        intakeButton
            .whileHeld(new ParallelCommandGroup(
                new Intake(intakeSubsystem),
                new SorterIn(sorterSubsystem)
            ))
            .whenReleased(new ParallelCommandGroup(
                new StopIntake(intakeSubsystem),
                new SorterStop(sorterSubsystem)
            ));

        outtakeButton
            .whileHeld(new Outtake(intakeSubsystem))
            .whenReleased(new StopIntake(intakeSubsystem));

        retractIntakeButton
            .whenPressed(new RetractIntake(intakeSubsystem));

        shiftLowGearButton
            .whenPressed(new ShiftLowGear(driveSubsystem));

        shiftHighGearButton
            .whenPressed(new ShiftHighGear(driveSubsystem));

        autoClimbButton
            .whenPressed(new AutoClimb(climberSubsystem));
    }

    private void configureCopilotButtonBindings() {

        JoystickButton toggleShooterButton = new JoystickButton(copilotGamepad, 3);

        JoystickButton shootNearButton = new JoystickButton(copilotGamepad, 2);
        JoystickButton shootMediumButton = new JoystickButton(copilotGamepad, 1);
        JoystickButton shootFarButton = new JoystickButton(copilotGamepad, 4);

        JoystickButton driveByLeftButton = new JoystickButton(copilotGamepad, 9);
        JoystickButton driveByRightButton = new JoystickButton(copilotGamepad, 10);

        JoystickButton tiltArmsButton = new JoystickButton(copilotGamepad, 12);

        Trigger extendArmsTrigger = new Trigger(() -> copilotGamepad.getRightY() < -0.5);
        Trigger retractArmsTrigger = new Trigger(() -> copilotGamepad.getRightY() > 0.5);

        JoystickButton shooterFeedUpButton = new JoystickButton(copilotGamepad, 6);
        JoystickButton shooterFeedDownButton = new JoystickButton(copilotGamepad, 8);

        JoystickButton sorterInButton = new JoystickButton(copilotGamepad, 5);
        JoystickButton sorterOutButton = new JoystickButton(copilotGamepad, 7);

        JoystickButton goHomeButton = new JoystickButton(copilotGamepad, 11);

        Trigger enableAutoAimButton = new Trigger(() -> copilotGamepad.getPOV() == 180);
        Trigger disableAutoAimButton = new Trigger(() -> copilotGamepad.getPOV() == 0);

        toggleShooterButton
            .whenPressed(new ToggleShooter(shooterSubsystem));

        shootNearButton
            .whenPressed(new ShootNear(shooterSubsystem, turretSubsystem));

        shootMediumButton
            .whenPressed(new ShootMedium(shooterSubsystem, turretSubsystem));

        shootFarButton
            .whenPressed(new ShootFar(shooterSubsystem, turretSubsystem));

        // enableClimberButton1.and(enableClimberButton2)
        //     .whenActive(new ToggleClimberEnabled(climberSubsystem));

        tiltArmsButton
            .whenPressed(new ToggleArmPositions(climberSubsystem));

        extendArmsTrigger
            .whenActive(new ExtendArms(climberSubsystem))
            .whenInactive(new StopArms(climberSubsystem));

        retractArmsTrigger
            .whenActive(new RetractArms(climberSubsystem))
            .whenInactive(new StopArms(climberSubsystem));

        shooterFeedUpButton
            .whenPressed(new ParallelCommandGroup(
                new SorterIn(sorterSubsystem),
                new ShooterFeedUp(shooterFeedSubsystem)
            ))
            .whenReleased(new ParallelCommandGroup(
                new SorterStop(sorterSubsystem),
                new ShooterFeedStop(shooterFeedSubsystem)
            ));

        shooterFeedDownButton
            .whenPressed(new ShooterFeedDown(shooterFeedSubsystem))
            .whenReleased(new ShooterFeedStop(shooterFeedSubsystem));

        sorterInButton
            .whenPressed(new SorterIn(sorterSubsystem))
            .whenReleased(new SorterStop(sorterSubsystem));

        sorterOutButton
            .whileHeld(new SorterOut(sorterSubsystem), false)
            .whenReleased(new SorterStop(sorterSubsystem));

        goHomeButton
            .whenPressed(new MoveTurretToDegrees(180, turretSubsystem));

        enableAutoAimButton
            .whenActive(new EnableAutoAim(turretSubsystem));

        disableAutoAimButton
            .whenActive(new DisableAutoAim(turretSubsystem));

        driveByLeftButton
            .whenPressed(new MoveTurretToDegrees(90, turretSubsystem));

        driveByRightButton
            .whenPressed(new MoveTurretToDegrees(270, turretSubsystem));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
