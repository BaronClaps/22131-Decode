package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.commands.FollowPath;
import org.firstinspires.ftc.teamcode.config.commands.ShootClose;
import org.firstinspires.ftc.teamcode.config.paths.Artifact12;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.util.OpModeCommand;

@Autonomous(name = "Blue 12", group = "Interesting")
public class BlueTwelve extends OpModeCommand {
    Robot r;

    @Override
    public void initialize() {
        r = new Robot(hardwareMap, Alliance.BLUE);
        Artifact12 p = new Artifact12(r);
        r.f.setStartingPose(p.start);

        r.s.down();
        r.i.in();

        schedule(
                new RunCommand(r::periodic),
                new RunCommand(() -> r.t.periodicError(r.l.angleFromShoot())),
                new SequentialCommandGroup(
                        new WaitCommand(1000),
                        new FollowPath(r, p.next())
                                .andThen(
                                        new ShootClose(r),
                                        new ShootClose(r),
                                        new ShootClose(r)
                                )/*,
                        new IntakeIn(r)
                                .alongWith(
                                        new FollowPath(r, p.next())
                                ),
                       new FollowPath(r, p.next())
                                .alongWith(
                                        new ShootClose(r)
                                                .andThen(
                                                        new ShootClose(r),
                                                        new ShootClose(r)
                                                )
                                ),
                        new IntakeIn(r)
                                .alongWith(
                                        new FollowPath(r, p.next())
                                ),
                       new FollowPath(r, p.next())
                                .alongWith(
                                        new ShootClose(r)
                                                .andThen(
                                                        new ShootClose(r),
                                                        new ShootClose(r)
                                                )
                                ),
                        new FollowPath(r, p.next()),
                        new IntakeIn(r)
                                .alongWith(
                                        new FollowPath(r, p.next())
                                ),
                       new FollowPath(r, p.next())
                                .alongWith(
                                        new ShootClose(r)
                                                .andThen(
                                                        new ShootClose(r),
                                                        new ShootClose(r)
                                                )
                                ),
                        new IntakeIn(r)
                                .alongWith(
                                        new FollowPath(r, p.next())
                                ),
                       new FollowPath(r, p.next())
                                .alongWith(
                                        new ShootClose(r)
                                                .andThen(
                                                        new ShootClose(r),
                                                        new ShootClose(r)
                                                )
                                ),
                        new IntakeIn(r)
                                .alongWith(
                                        new FollowPath(r, p.next())
                                ),
                       new FollowPath(r, p.next())
                                .alongWith(
                                        new ShootClose(r)
                                                .andThen(
                                                        new ShootClose(r),
                                                        new ShootClose(r)
                                                )
                                ),
                        new IntakeIn(r)
                                .alongWith(
                                        new FollowPath(r, p.next())
                                ),
                        new ShootClose(r)
                                .alongWith(
                                        new FollowPath(r, p.next())
                                )*/
                )
        );
    }

    @Override
    public void start() {
        r.s.close();
        r.i.spinIdle();
    }

    @Override
    public void stop() {
        r.stop();
    }
}
