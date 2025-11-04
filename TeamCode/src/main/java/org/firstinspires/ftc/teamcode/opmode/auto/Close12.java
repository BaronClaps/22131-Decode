package org.firstinspires.ftc.teamcode.opmode.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.commands.FollowPath;
import org.firstinspires.ftc.teamcode.config.commands.IntakeIn;
import org.firstinspires.ftc.teamcode.config.commands.ShootClose;
import org.firstinspires.ftc.teamcode.config.paths.NoCorner12;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.util.OpModeCommand;

@Autonomous(name = "Close 12", group = "Interesting")
public class Close12 extends OpModeCommand {
    Robot r;
    public static Pose shootTarget = new Pose(6, 144 - 6, 0);

    @Override
    public void initialize() {
        r = new Robot(hardwareMap, Alliance.BLUE);
        NoCorner12 p = new NoCorner12(r);
        r.f.setStartingPose(p.start);

        r.s.down();
        r.t.resetTurret();

        schedule(
                new RunCommand(r::periodic),
                new RunCommand(() -> r.t.face(shootTarget, r.f.getPose())),
                new RunCommand(() -> {
                    telemetry.addData("at target", r.s.atTarget());
                    telemetry.update();
                }),
                new RunCommand(() -> r.t.face(shootTarget, r.f.getPose())),
                new SequentialCommandGroup(
                        r.i.in(),
                        new WaitCommand(500),
                        new FollowPath(r, p.next())
                                .andThen(
                                        new WaitCommand(500),
                                        new ShootClose(r),
                                        new ShootClose(r),
                                        new ShootClose(r)
                                )
                        ,
                        new IntakeIn(r)
                                .alongWith(
                                        new FollowPath(r, p.next())
                                ),
                        new FollowPath(r, p.next()),
                        new WaitCommand(3000),
                        new FollowPath(r, p.next())
                                .andThen(
                                        new WaitCommand(500)
                                                .andThen(
                                                        new ShootClose(r),
                                                        new ShootClose(r),
                                                        new ShootClose(r)
                                                )
                                ),
                        new IntakeIn(r)
                                .alongWith(
                                        new FollowPath(r, p.next())
                                ),
                        new FollowPath(r, p.next())
                                .andThen(
                                        new WaitCommand(500)
                                                .andThen(
                                                        new ShootClose(r),
                                                        new ShootClose(r),
                                                        new ShootClose(r)
                                                )
                                )
                )
        );
    }

    @Override
    public void start() {
    }

    @Override
    public void stop() {
        r.stop();
    }
}
