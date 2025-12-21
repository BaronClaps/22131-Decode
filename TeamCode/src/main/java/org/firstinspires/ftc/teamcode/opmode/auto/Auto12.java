package org.firstinspires.ftc.teamcode.opmode.auto;

import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.groups.Sequential;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.command.FollowPath;
import org.firstinspires.ftc.teamcode.config.paths.Slow12;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.command.CommandOpMode;

public abstract class Auto12 extends CommandOpMode {
    Robot r;
    final Alliance a;

    public Auto12(Alliance alliance) {
        a = alliance;
    }

    @Override
    public void init() {
        r = new Robot(hardwareMap, a);
        Slow12 p = new Slow12(r);
        r.setStart(p.start);

        r.g.closeGate();
        r.t.resetTurret();

        reset();

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
                        new FollowPath(r, p.next()),
                        r.shoot(),
                        new FollowPath(r, p.next()),
                        r.intake()
                                .with(
                                        new FollowPath(r, p.next())
                                ),
                        new FollowPath(r, p.next()),
                        r.i.idle(),
                        new FollowPath(r, p.next()),
                        new Wait(500),
                        new FollowPath(r, p.next()),
                        
                        r.shoot(),
                        new FollowPath(r, p.next()),
                        r.intake()
                                .with(
                                        new FollowPath(r, p.next())
                                ),
                        new FollowPath(r, p.next()),
                        
                        r.shoot(),
                        new FollowPath(r, p.next()),
                        r.intake()
                                .with(
                                        new FollowPath(r, p.next())
                                ),
                        new FollowPath(r, p.next()),
                        
                        r.shoot(),
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
