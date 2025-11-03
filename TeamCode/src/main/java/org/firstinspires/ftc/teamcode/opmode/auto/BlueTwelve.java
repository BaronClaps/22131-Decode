package org.firstinspires.ftc.teamcode.opmode.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.commands.FollowPath;
import org.firstinspires.ftc.teamcode.config.commands.IntakeIn;
import org.firstinspires.ftc.teamcode.config.commands.ShootClose;
import org.firstinspires.ftc.teamcode.config.paths.Artifact12;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.util.OpModeCommand;

@Autonomous(name = "Blue 12", group = "Interesting")
public class BlueTwelve extends OpModeCommand {
    Robot r;
    public static Pose shootTarget = new Pose(8, 144-8, 0);

    @Override
    public void initialize() {
        r = new Robot(hardwareMap, Alliance.BLUE);
        Artifact12 p = new Artifact12(r);
        r.f.setStartingPose(p.start);

        r.s.down();
        r.t.resetTurret();
        r.i.spinIdle();

        schedule(
                new RunCommand(r::periodic),
                new RunCommand(() -> r.t.face(shootTarget, r.f.getPose())),
                //new RunCommand(() -> r.s.forDistance(shootTarget.distanceFrom(r.f.getPose()))),
                new RunCommand(() -> { telemetry.addData("at target", r.s.atTarget()); telemetry.update(); }),
                new SequentialCommandGroup(
                        r.i.in(),
                        new WaitCommand(500),
                        //new InstantCommand(() -> r.i.set(0.5)),
                        new FollowPath(r, p.next()),
                        new SequentialCommandGroup(
                        new InstantCommand(() -> r.f.breakFollowing()),
                      //  new InstantCommand(() -> r.s.close())
                        new WaitCommand(500),
                        new ShootClose(r),
                        new ShootClose(r),
                        new ShootClose(r)
                        ).alongWith(new RunCommand(() -> r.t.face(shootTarget, r.f.getPose()))
                        )
                        /*,
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
    }

    @Override
    public void stop() {
        r.stop();
    }
}
