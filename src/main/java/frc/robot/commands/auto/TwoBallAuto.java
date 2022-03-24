package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.FollowPath;
import frc.robot.commands.drive.ResetOdometry;
import frc.robot.commands.intake.AutoIntake;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.shooter.ShootMedium;
import frc.robot.commands.shooter.ShooterOff;
import frc.robot.commands.shooterfeed.ShooterFeedStop;
import frc.robot.commands.shooterfeed.AutoShooterFeedUp;
import frc.robot.commands.sorter.SorterIn;
import frc.robot.commands.sorter.SorterStop;
import frc.robot.pathfollowing.PathType;

public class TwoBallAuto extends SequentialCommandGroup {

    public TwoBallAuto() {
        addCommands(
            new ResetOdometry(10.3, 2.75, 200),
            new AutoIntake(),
            new WaitCommand(1),
            new SorterIn(),
            new ShootMedium(),
            new WaitCommand(1),
            new FollowPath(PathType.TwoBall1),
            new AutoShooterFeedUp(),
            new WaitCommand(2),
            new RetractIntake(),
            new WaitCommand(2),
            new ShooterFeedStop(),
            new SorterStop(),
            new ShooterOff()
        );
    }
}