package org.firstinspires.ftc.teamcode.opmode.auto;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Sequential;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.commands.FollowPath;
import org.firstinspires.ftc.teamcode.config.commands.IntakeIn;
import org.firstinspires.ftc.teamcode.config.commands.Shoot;
import org.firstinspires.ftc.teamcode.config.paths.NoCorner12;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.util.OpModeCommand;

@Autonomous(name = "Blue 12", group = "Interesting", preselectTeleOp = "Tele")
public class Blue12 extends OpModeCommand {
    Robot r;
    Timer t;

    @Override
    public void init() {
        r = new Robot(hardwareMap, Alliance.BLUE);
        NoCorner12 p = new NoCorner12(r);
        r.f.setStartingPose(p.start);

        r.s.down();
        r.t.resetTurret();

        t = new Timer();

        schedule(
                new Infinite(r::periodic),
                new Infinite(() -> {
                    double dist = r.getShootTarget().distanceFrom(r.f.getPose());
                    r.s.forDistance(dist, true);
                }),
                new Infinite(() -> {
                    telemetry.addData("Pose: ", r.f.getPose());
                    telemetry.addData("Follower Busy: ", r.f.isBusy());
                    telemetry.addData("Shooter At Target: ", r.s.atTarget());
                    telemetry.addData("Flipper at Up: ", r.s.atUp());
                    telemetry.addData("Turret Ticks: ", r.t.getTurret());
                    telemetry.addData("Turret Ready: ", r.t.isReady());
                    telemetry.addData("Shooter At Target: ", r.s.atTarget());
                    telemetry.addData("Shooter Velocity: ", r.s.getVelocity());
                    telemetry.addData("Shooter Target", r.s.getTarget());
                    telemetry.update();
                }),
                new Sequential(
                        new Wait(1),
                        r.i.in(),
                        new Instant(() -> r.t.face(r.getShootTarget(), p.score)),
                        new FollowPath(r, p.next())
                                .with(
                                        new Wait(500)
                                                .then(
                                                        new WaitUntil(() -> r.f.getCurrentTValue() >= 0.5)
                                                                .then(
                                                                        new Instant(() -> r.s.on())
                                                                )
                                                )
                                ),
                        new Shoot(r),
                        new Wait(250),
                        new IntakeIn(r)
                                .with(
                                        new FollowPath(r, p.next())
                                ),
                        new FollowPath(r, p.next())
                                .with(
                                        new Wait(500)
                                                .then(
                                                        new WaitUntil(() -> r.f.getCurrentTValue() >= 0.5)
                                                                .then(
                                                                        new Instant(() -> r.s.on())
                                                                )
                                                )
                                ),
                        new Shoot(r),
                        new IntakeIn(r)
                                .with(
                                        new FollowPath(r, p.next())
                                ),
                        new FollowPath(r, p.next()),
                        r.i.stop(),
                        new Wait(250),
                        new FollowPath(r, p.next())
                                .with(
                                        new Wait(500)
                                                .then(
                                                        new WaitUntil(() -> r.f.getCurrentTValue() >= 0.5)
                                                                .then(
                                                                        new Instant(() -> r.s.on())
                                                                )
                                                )
                                ),
                        new Shoot(r),
                        new Wait(250),
                        new IntakeIn(r)
                                .with(new FollowPath(r, p.next())),
                        new Instant(() -> r.t.face(r.getShootTarget(), p.scoreCorner)),
                        new FollowPath(r, p.next())
                                .with(
                                        new Wait(500)
                                                .then(
                                                        new WaitUntil(() -> r.f.getCurrentTValue() >= 0.5)
                                                                .then(
                                                                        new Instant(() -> r.s.on())
                                                                )
                                                )
                                ),
                        new WaitUntil(() -> r.t.isReady()),
                        new Shoot(r),
                        new IntakeIn(r)
                                .with(new FollowPath(r, p.next())),
                        new FollowPath(r, p.next())
                                .with(
                                        new Wait(2000)
                                                .then(
                                                        new WaitUntil(() -> r.f.getCurrentTValue() >= 0.5)
                                                                .then(
                                                                        new Instant(() -> r.s.on())
                                                                )
                                                )
                                ),
                        new Shoot(r),
                        new FollowPath(r, p.next())
                )
        );
    }

    @Override
    public void stop() {
        r.stop();
        reset();
    }
}
