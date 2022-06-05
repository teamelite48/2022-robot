package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.FollowPath;
import frc.robot.commands.drive.ResetOdometry;
import frc.robot.commands.intake.AutoIntake;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.shooter.AutoShoot;
import frc.robot.commands.shooter.ShooterOff;
import frc.robot.commands.shooterfeed.ShooterFeedUp;
import frc.robot.commands.sorter.SorterIn;
import frc.robot.pathfollowing.PathType;

public class TwoBallAuto extends SequentialCommandGroup {

    public TwoBallAuto() {
        addCommands(
            new ResetOdometry(10.3, 2.75, 200),
            new AutoIntake(),
            new SorterIn(),
            new WaitCommand(1),
            new FollowPath(PathType.TwoBall1),
            new AutoShoot(),
            new WaitCommand(0.5),
            //new RetractIntake(),
            new ShooterFeedUp().withTimeout(2.5),
            new FollowPath(PathType.TwoBall2),
            new AutoShoot(),
            new WaitCommand(0.5),
            //new RetractIntake(),
            new ShooterFeedUp().withTimeout(3),
            new ShooterOff()
        );
    }
}