package org.firstinspires.ftc.teamcode.opmode.auto;

import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Sequential;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.paths.NoCorner12;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.util.OpModeCommand;

@Autonomous
public class RevAmpedPoisonIvyAutonomousSixSeven extends OpModeCommand {
    Robot r;

    @Override
    public void init() {
        r = new Robot(hardwareMap, Alliance.RED);
        NoCorner12 p = new NoCorner12(r);
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
                    telemetry.update();
                }),
                new Sequential(
                        new Wait(500),
                        new Instant(() -> r.t.set(Math.toRadians(90))),
                        new WaitUntil(() -> r.t.isReady()),
                        new Instant(() -> r.s.flip()),
                        new Wait(500),
                        new Instant(() -> r.t.set(Math.toRadians(-90))),
                        new WaitUntil(() -> r.t.isReady()),
                        new Instant(() -> r.s.flip()),
                        new Wait(500)
                )
        );

    }

    @Override
    public void stop() {
        r.stop();
        reset();
    }
}
