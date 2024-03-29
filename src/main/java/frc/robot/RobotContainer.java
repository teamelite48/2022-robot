// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Joysticks.LogitechJoystick;
import frc.robot.Joysticks.PS4Gamepad;
import frc.robot.commands.auto.BackOffLineAuto;
import frc.robot.commands.auto.FourBallStraightAuto;
import frc.robot.commands.auto.TwoBallAuto;
import frc.robot.commands.auto.TwoBallShortAuto;
import frc.robot.commands.climber.ToggleArmPositions;
import frc.robot.commands.climber.ToggleHookPositions;
import frc.robot.commands.climber.DisableClimber;
import frc.robot.commands.climber.EnableClimber;
import frc.robot.commands.climber.ToggleArmLocks;
import frc.robot.commands.drive.ShiftHighGear;
import frc.robot.commands.drive.ShiftLowGear;
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
import frc.robot.config.subsystems.ClimberConfig;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterFeedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SorterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

    public static DriveSubsystem driveSubsystem;
    public static IntakeSubsystem intakeSubsystem;
    public static ShooterSubsystem shooterSubsystem;
    public static ClimberSubsystem climberSubsystem;
    public static ShooterFeedSubsystem shooterFeedSubsystem;
    public static SorterSubsystem sorterSubsystem;
    public static TurretSubsystem turretSubsystem;

    final LogitechJoystick leftJoystick = new LogitechJoystick(JoystickPort.LeftPilotJoystick);
    final LogitechJoystick rightJoystick = new LogitechJoystick(JoystickPort.RightPilotJoystick);
    final PS4Gamepad gamepad = new PS4Gamepad(JoystickPort.CopilotGamepad);

    final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {

        driveSubsystem = new DriveSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        shooterSubsystem = new ShooterSubsystem();
        climberSubsystem = new ClimberSubsystem();
        shooterFeedSubsystem = new ShooterFeedSubsystem();
        sorterSubsystem = new SorterSubsystem();
        turretSubsystem = new TurretSubsystem();

        driveSubsystem.setDefaultCommand(
            new RunCommand(() -> driveSubsystem.tankDrive(-leftJoystick.getY(), -rightJoystick.getY(), leftJoystick.getRawAxis(2)), driveSubsystem)
        );

        configurePilotButtonBindings();
        configureCopilotButtonBindings();

        initializeCamera();
        inititialzeAutoChooser();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void configurePilotButtonBindings() {

        leftJoystick.getTrigger().whenHeld(new ManualIntake());
        leftJoystick.getButton4().whenPressed(new ShiftLowGear());
        leftJoystick.getButton5().whenPressed(new ShiftHighGear());

        rightJoystick.getTrigger().whenPressed(new RetractIntake());
        rightJoystick.getButton2().whenHeld(new Outtake());

        rightJoystick.getButton8()
            .and(rightJoystick.getButton9())
            .whenActive(new ConditionalCommand(new DisableClimber(), new EnableClimber(), climberSubsystem::isEnabled));
    }

    private void configureCopilotButtonBindings() {

        gamepad.getL1Button().whenPressed(new BumpShooterRpmUp());
        gamepad.getL2Button().whenPressed(new BumpShooterRpmDown());

        gamepad.getR1Button().whenHeld(new ShooterFeedUp());
        gamepad.getR2Button().whenHeld(new ShooterFeedDown());

        gamepad.getDpadUpTrigger().whenActive(new MoveTurretToDegrees(180));
        gamepad.getDpadDownTrigger().whenActive(new AutoAimOn());
        gamepad.getDpadLeftTrigger().whileActiveOnce(new RotateTurretCounterClockwise());
        gamepad.getDpadRightTrigger().whileActiveOnce(new RotateTurretClockwise());

        gamepad.getLeftStickButton().whenPressed(new ToggleArmPositions());
        gamepad.getRightStickButton().whenPressed(new ToggleHookPositions());

        gamepad.getTouchpadButton().whenPressed(new ToggleArmLocks());


        gamepad.getPSButton().whenPressed(new ConditionalCommand(new DisableClimber(), new EnableClimber(), climberSubsystem::isEnabled));

        new Trigger(() -> Math.abs(gamepad.getLeftY()) > ClimberConfig.armSpeedDeadband)
            .whileActiveContinuous(new InstantCommand(() -> climberSubsystem.moveArms(gamepad.getLeftY() * -1), climberSubsystem))
            .whenInactive(new InstantCommand(climberSubsystem::stopArms));

        gamepad.getBackButton().whenPressed(new DisableAutoAim());
        gamepad.getStartButton().whenPressed(new EnableAutoAim());

        gamepad.getCrossButton().whenPressed(new ShootNear());
        gamepad.getCircleButton().whenPressed(new ShooterOff());
        gamepad.getSquareButton().whenPressed(new ShootMedium());
        gamepad.getTriangleButton().whenPressed(new AutoShoot());
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
        autoChooser.addOption("Back Off Line Path", new BackOffLineAuto());
        autoChooser.addOption("Two Ball", new TwoBallAuto());
        autoChooser.addOption("Two Ball Short", new TwoBallShortAuto());
        autoChooser.addOption("Four Ball Straight", new FourBallStraightAuto());

        SmartDashboard.putData(autoChooser);
    }
}
