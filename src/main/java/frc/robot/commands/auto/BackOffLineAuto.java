package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.ResetOdometry;
import frc.robot.commands.shooter.ShootMedium;
import frc.robot.commands.shooter.ShooterOff;
import frc.robot.commands.shooter.ShooterOn;
import frc.robot.commands.shooterfeed.ShooterFeedStop;
import frc.robot.commands.shooterfeed.ShooterFeedUp;
import frc.robot.commands.sorter.SorterIn;
import frc.robot.commands.sorter.SorterStop;
import frc.robot.pathfollowing.RamseteCommandFactory;
import frc.robot.pathfollowing.TrajectoryType;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterFeedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SorterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class BackOffLineAuto extends SequentialCommandGroup {

    public BackOffLineAuto(
        DriveSubsystem driveSubsystem,
        SorterSubsystem sorterSubsystem,
        ShooterSubsystem shooterSubsystem,
        TurretSubsystem turretSubsystem,
        ShooterFeedSubsystem shooterFeedSubsystem
    ) {
        RamseteCommandFactory ramseteCommandFactory = new RamseteCommandFactory(driveSubsystem);

        addCommands(
            new ResetOdometry(10.5, 3.45, 180, driveSubsystem),
            new ShootMedium(shooterSubsystem, turretSubsystem),
            new ShooterOn(shooterSubsystem),
            ramseteCommandFactory.createCommand(TrajectoryType.BackOffLine),
            new SorterIn(sorterSubsystem),
            new ShooterFeedUp(shooterFeedSubsystem),
            new WaitCommand(2),
            new ShooterFeedStop(shooterFeedSubsystem),
            new SorterStop(sorterSubsystem),
            new ShooterOff(shooterSubsystem)
        );
    }
}
