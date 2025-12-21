package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.command.ButtonMapper;
import org.firstinspires.ftc.teamcode.config.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.config.command.GamepadEx;
import org.firstinspires.ftc.teamcode.config.util.Alliance;

@Config
public class Tele extends CommandOpMode {
    final Alliance a;
    boolean shoot, manualAim;
    public static double shootTarget = 1200;

    public Tele(Alliance alliance) {
        a = alliance;
    }

    @Override
    public void init() {
        Robot r = new Robot(hardwareMap, a);
        GamepadEx g = new GamepadEx(gamepad1);

        ButtonMapper m = new ButtonMapper()
                .put(g.right_trigger.greaterThan(0.25F), r.shoot().then(new Instant(r.d::start)))
                .put(g.left_trigger.greaterThan(0.25F), toggleAim())
                .put(g.a.risingEdge(), toggleManual())
                .put(g.b.risingEdge(), r.d.toggleCentric())
                .put(g.x.risingEdge(), r.t.reset())
                .put(g.y.risingEdge(), r.t.set(0))
                .put(g.left_bumper.risingEdge(), r.i.toggleOut())
                .put(g.right_bumper.risingEdge(), r.i.toggleIn())
                .put(g.dpad_down.risingEdge(), r.d.corner())
                .put(g.dpad_left, r.t.left(manualAim))
                .put(g.dpad_right, r.t.right(manualAim));

        schedule(
                new Sequential(
                        new Wait(1),
                        new Instant(() -> {
                            r.periodic();
                            r.d.start();
                            r.d.setGamepad(gamepad1);
                        }),
                        new Infinite(() -> {
                            r.periodic();
                            autoAim(r);
                            m.update();
                        })
                )
        );
    }

    public Instant toggleAim() {
        return new Instant(() -> shoot = !shoot);
    }

    public Instant toggleManual() {
        return new Instant(() -> manualAim = !manualAim);
    }

    public void autoAim(Robot r) {
        if (shoot) {
            r.s.on();
            r.t.on();

            if (manualAim) {
                r.s.setTarget(shootTarget);
            } else {
                double dist = r.getShootTarget().distanceFrom(r.d.getPose());
                telemetry.addData("dist", dist);
                boolean close = r.d.getPose().getY() > 48;
                r.s.forDistance(dist, close);
                r.t.face(r.getShootTarget(), r.d.getPose());
                r.t.automatic();
            }
        } else {
            r.s.off();
            r.t.off();
        }
    }
}
