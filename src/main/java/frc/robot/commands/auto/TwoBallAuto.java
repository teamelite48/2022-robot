package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.ResetOdometry;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.shooter.ShooterOff;
import frc.robot.commands.shooter.ShooterOn;
import frc.robot.commands.shooterfeed.ShooterFeedStop;
import frc.robot.commands.shooterfeed.ShooterFeedUp;
import frc.robot.commands.sorter.SorterIn;
import frc.robot.commands.sorter.SorterStop;
import frc.robot.pathfollowing.RamseteCommandFactory;
import frc.robot.pathfollowing.TrajectoryType;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterFeedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SorterSubsystem;

public class TwoBallAuto extends SequentialCommandGroup {


    public TwoBallAuto(
        DriveSubsystem driveSubsystem,
        IntakeSubsystem intakeSubsystem,
        SorterSubsystem sorterSubsystem,
        ShooterSubsystem shooterSubsystem,
        ShooterFeedSubsystem shooterFeedSubsystem
    ) {

        RamseteCommandFactory ramseteCommandFactory = new RamseteCommandFactory(driveSubsystem);

        addCommands(
            new ResetOdometry(10.5, 3.1, -136, driveSubsystem),
            new Intake(intakeSubsystem),
            ramseteCommandFactory.createCommand(TrajectoryType.TwoBall1),
            new SorterIn(sorterSubsystem),
            new RetractIntake(intakeSubsystem),
            new ShooterOn(shooterSubsystem),
            ramseteCommandFactory.createCommand(TrajectoryType.TwoBall2),
            new ShooterFeedUp(shooterFeedSubsystem),
            new WaitCommand(2),
            new SorterStop(sorterSubsystem),
            new ShooterFeedStop(shooterFeedSubsystem),
            new ShooterOff(shooterSubsystem)

        );
    }
}