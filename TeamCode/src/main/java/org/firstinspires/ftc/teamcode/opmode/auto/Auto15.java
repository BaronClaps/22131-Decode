package org.firstinspires.ftc.teamcode.opmode.auto;

import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Sequential;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.commands.FollowPath;
import org.firstinspires.ftc.teamcode.config.commands.IntakeIn;
import org.firstinspires.ftc.teamcode.config.commands.Shoot;
import org.firstinspires.ftc.teamcode.config.paths.Fast15;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.util.OpModeCommand;

public abstract class Auto15 extends OpModeCommand {
    Robot r;
    final Alliance a;

    public Auto15(Alliance alliance) {
        a =  alliance;
    }

    @Override
    public void init() {
        r = new Robot(hardwareMap, a);
        Fast15 p = new Fast15(r);
        r.f.setStartingPose(p.start);

        r.s.down();
        r.t.resetTurret();

        schedule(
                new Infinite(r::periodic),
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

                                        new WaitUntil(() -> r.f.getCurrentTValue() >= 0.5)
                                                .then(
                                                        new Shoot(r)
                                                )

                                ),
                        new IntakeIn(r)
                                .with(
                                        new FollowPath(r, p.next())
                                ),
                        new FollowPath(r, p.next())
                                .with(

                                        new WaitUntil(() -> r.f.getCurrentTValue() >= 0.5)
                                                .then(
                                                        new Shoot(r)
                                                )

                                ),
                        new IntakeIn(r)
                                .with(
                                        new FollowPath(r, p.next())
                                ),
                        new FollowPath(r, p.next()),
                        r.i.idle(),
                        new Wait(250),
                        new FollowPath(r, p.next())
                                .with(

                                        new WaitUntil(() -> r.f.getCurrentTValue() >= 0.5)
                                                .then(
                                                        new Shoot(r)
                                                )

                                ),
                        new IntakeIn(r)
                                .with(new FollowPath(r, p.next())),
                        new Instant(() -> r.t.face(r.getShootTarget(), p.scoreCorner)),
                        new FollowPath(r, p.next())
                                .with(

                                        new WaitUntil(() -> r.f.getCurrentTValue() >= 0.5)
                                                .then(
                                                        new Instant(() -> r.s.close())
                                                )

                                ),
                        new WaitUntil(() -> r.t.isReady()),
                        new Shoot(r),
                        new IntakeIn(r)
                                .with(new FollowPath(r, p.next())),
                        new FollowPath(r, p.next())
                                .with(
                                        new WaitUntil(() -> r.f.getCurrentTValue() >= 0.5)
                                                .then(
                                                        new Shoot(r)
                                                )
                                ),
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
