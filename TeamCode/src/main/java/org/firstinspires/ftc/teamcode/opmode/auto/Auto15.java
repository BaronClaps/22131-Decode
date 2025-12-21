package org.firstinspires.ftc.teamcode.opmode.auto;

import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Sequential;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.command.FollowPath;
import org.firstinspires.ftc.teamcode.config.paths.Fast15;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.command.CommandOpMode;

public abstract class Auto15 extends CommandOpMode {
    Robot r;
    final Alliance a;

    public Auto15(Alliance alliance) {
        a =  alliance;
    }

    @Override
    public void init() {
        r = new Robot(hardwareMap, a);
        Fast15 p = new Fast15(r);
        r.setStart(p.start);

        r.g.closeGate();
        r.t.resetTurret();

        schedule(
                new Infinite(r::periodic),
                new Infinite(() -> {
                    r.t.face(r.getShootTarget(), r.d.getPose());
                    telemetry.addData("Pose: ", r.d.getPose());
                    telemetry.addData("Follower Busy: ", r.d.isBusy());
                    telemetry.addData("Shooter At Target: ", r.s.atTarget());
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
                        new FollowPath(r, p.next())
                                .with(

                                        new WaitUntil(() -> r.d.getT() >= 0.5)
                                                .then(
                                                        r.shoot()
                                                )

                                ),
                        r.intake()
                                .with(
                                        new FollowPath(r, p.next())
                                ),
                        new FollowPath(r, p.next())
                                .with(

                                        new WaitUntil(() -> r.d.getT() >= 0.5)
                                                .then(
                                                        r.shoot()
                                                )

                                ),
                        r.intake()
                                .with(
                                        new FollowPath(r, p.next())
                                ),
                        new FollowPath(r, p.next()),
                        r.i.idle(),
                        new Wait(250),
                        new FollowPath(r, p.next())
                                .with(
                                        new WaitUntil(() -> r.d.getT() >= 0.5)
                                                .then(
                                                        r.shoot()
                                                )

                                ),
                        r.intake()
                                .with(new FollowPath(r, p.next())),
                        //new Instant(() -> r.t.face(r.getShootTarget(), p.scoreCorner)),
                        new FollowPath(r, p.next())
                                .with(

                                        new WaitUntil(() -> r.d.getT() >= 0.5)
                                                .then(
                                                        new Instant(() -> r.s.shootNear())
                                                )

                                ),
                        new WaitUntil(() -> r.t.isReady()),
                        r.shoot(),
                        r.intake()
                                .with(new FollowPath(r, p.next())),
                        new FollowPath(r, p.next())
                                .with(
                                        new WaitUntil(() -> r.d.getT() >= 0.5)
                                                .then(
                                                        r.shoot()
                                                )
                                ),
                        new FollowPath(r, p.next())
                )
        );
    }

    @Override
    public void stop() {
        r.saveEnd();
        reset();
    }
}
