// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Joysticks.LogitechD;
import frc.robot.Joysticks.LogitechX;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.drive.DriveBackwardsCommand;
import frc.robot.commands.intake.ManualIntake;
import frc.robot.commands.intake.Outtake;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.shooter.ShootNear;
import frc.robot.commands.shooter.ShooterOff;
import frc.robot.commands.shooter.AutoShoot;
import frc.robot.commands.shooter.BumpShooterRpmDown;
import frc.robot.commands.shooter.BumpShooterRpmUp;
import frc.robot.commands.shooter.ShootMedium;
import frc.robot.commands.shooterfeed.ShooterFeedDown;
import frc.robot.commands.shooterfeed.ShooterFeedUp;
import frc.robot.commands.turret.AutoAimOn;
import frc.robot.commands.turret.DisableAutoAim;
import frc.robot.commands.turret.EnableAutoAim;
import frc.robot.commands.turret.MoveTurretToDegrees;
import frc.robot.commands.turret.RotateTurretClockwise;
import frc.robot.commands.turret.RotateTurretCounterClockwise;
import frc.robot.config.roborio.JoystickPort;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterFeedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SorterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RobotContainer {

    public static DrivetrainSubsystem drivetrainSubsystem;
    public static IntakeSubsystem intakeSubsystem;
    public static ShooterSubsystem shooterSubsystem;
    public static ShooterFeedSubsystem shooterFeedSubsystem;
    public static SorterSubsystem sorterSubsystem;
    public static TurretSubsystem turretSubsystem;

    final LogitechD pilotGamepad = new LogitechD(JoystickPort.PilotGamepad);
    final LogitechX copilotGamepad = new LogitechX(JoystickPort.CopilotGamepad);

    final SendableChooser<Command> autoChooser = new SendableChooser<>();

    final int slewRate = 2;

    final SlewRateLimiter rateLimiterX = new SlewRateLimiter(slewRate);
    final SlewRateLimiter rateLimiterY = new SlewRateLimiter(slewRate);
    final SlewRateLimiter rateLimiterRotation = new SlewRateLimiter(slewRate);

    public RobotContainer() {

        drivetrainSubsystem = new DrivetrainSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        shooterSubsystem = new ShooterSubsystem();
        shooterFeedSubsystem = new ShooterFeedSubsystem();
        sorterSubsystem = new SorterSubsystem();
        turretSubsystem = new TurretSubsystem();

        // drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        //     drivetrainSubsystem,
        //     () -> rateLimiterY.calculate(pilotGamepad.getLeftYAxis()),
        //     () -> rateLimiterX.calculate(pilotGamepad.getLeftXAxis()),
        //     () -> rateLimiterRotation.calculate(pilotGamepad.getRightXAxis())
        // ));

        configurePilotButtonBindings();
        configureCopilotButtonBindings();

        initializeCamera();
        inititialzeAutoChooser();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void configurePilotButtonBindings() {
        pilotGamepad.lb.whenHeld(new Outtake());
        pilotGamepad.rb.whenHeld(new ManualIntake());

        pilotGamepad.a.whenPressed(new RetractIntake());
        pilotGamepad.start.whenPressed(drivetrainSubsystem::zeroGyroscope);
     }

    private void configureCopilotButtonBindings() {

        copilotGamepad.lb.whenPressed(new BumpShooterRpmUp());
        copilotGamepad.lt.whileActiveOnce(new BumpShooterRpmDown());

        copilotGamepad.rb.whenHeld(new ShooterFeedUp());
        copilotGamepad.rt.whileActiveContinuous(new ShooterFeedDown());

        copilotGamepad.up.whenActive(new MoveTurretToDegrees(180));
        copilotGamepad.down.whenActive(new AutoAimOn());
        copilotGamepad.left.whileActiveOnce(new RotateTurretCounterClockwise());
        copilotGamepad.right.whileActiveOnce(new RotateTurretClockwise());

        copilotGamepad.back.whenPressed(new DisableAutoAim());
        copilotGamepad.start.whenPressed(new EnableAutoAim());

        copilotGamepad.a.whenPressed(new ShootNear());
        copilotGamepad.b.whenPressed(new ShooterOff());
        copilotGamepad.x.whenPressed(new ShootMedium());
        copilotGamepad.y.whenPressed(new AutoShoot());
    }

    private void initializeCamera(){

        if (RobotBase.isSimulation()) return;

        UsbCamera backCam = CameraServer.startAutomaticCapture();
        backCam.setResolution(160, 120);
        backCam.setFPS(20);
        backCam.setExposureAuto();
    }

    private void inititialzeAutoChooser() {

        autoChooser.setDefaultOption("Do Nothing", new WaitCommand(3));

        autoChooser.addOption("Back Up & Shoot", new SequentialCommandGroup(
            new DriveBackwardsCommand(1),
            new AutoShoot(),
            new WaitCommand(2),
            new ShooterFeedUp(),
            new WaitCommand(3),
            new ShooterOff()
        ));

        SmartDashboard.putData(autoChooser);
    }
}
