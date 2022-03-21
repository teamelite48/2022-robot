package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.FollowPath;
import frc.robot.commands.drive.ResetOdometry;
import frc.robot.commands.shooter.ShootMedium;
import frc.robot.commands.shooter.ShooterOff;
import frc.robot.commands.shooter.ShooterOn;
import frc.robot.commands.shooterfeed.ShooterFeedStop;
import frc.robot.commands.shooterfeed.AutoShooterFeedUp;
import frc.robot.commands.sorter.SorterIn;
import frc.robot.commands.sorter.SorterStop;
import frc.robot.pathfollowing.PathType;

public class BackOffLineAuto extends SequentialCommandGroup {

    public BackOffLineAuto() {
        addCommands(
            new ResetOdometry(10.5, 3.45, 180),
            new ShootMedium(),
            new ShooterOn(),
            new FollowPath(PathType.BackOffLine),
            new SorterIn(),
            new AutoShooterFeedUp(),
            new WaitCommand(2),
            new ShooterFeedStop(),
            new SorterStop(),
            new ShooterOff()
        );
    }
}