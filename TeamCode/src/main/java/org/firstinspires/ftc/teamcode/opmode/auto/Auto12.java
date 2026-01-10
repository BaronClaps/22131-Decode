package org.firstinspires.ftc.teamcode.opmode.auto;

import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.pedro.PedroCommands;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.paths.Slow12;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.command.CommandOpMode;

import static com.pedropathing.ivy.groups.Groups.sequential;

public abstract class Auto12 extends CommandOpMode {
    Robot r;
    final Alliance a;
    boolean angle = false;

    public Auto12(Alliance alliance) {
        a = alliance;
    }

    @Override
    public void init() {
        r = new Robot(hardwareMap, a);
        r.t.setYaw(0);
        Slow12 p = new Slow12(r);
        r.f.setStartingPose(p.start);

        r.g.flipDown();
        r.t.resetTurret();
        r.t.setPowerZero();

        reset();

        schedule(
                Commands.infinite(r::periodic),
                Commands.infinite(() -> {
                    if (angle)
                        r.t.face(r.getShootTarget(), r.f.getPose());
                    r.s.forPose(r.f.getPose(), r.getShootTarget(), true);
                    telemetry.addData("Pose: ", r.f.getPose());
                    telemetry.addData("Follower Busy: ", r.f.isBusy());
                    telemetry.addData("Shooter At Target: ", r.s.atTarget());
                    telemetry.addData("Turret Ticks: ", r.t.getTurret());
                    telemetry.addData("Turret Ready: ", r.t.isReady());
                    telemetry.addData("Shooter At Target: ", r.s.atTarget());
                    telemetry.addData("Shooter Velocity: ", r.s.getVelocity());
                    telemetry.addData("Shooter Target", r.s.getTarget());
                    telemetry.update();
                }),
                sequential(
                        Commands.instant(() -> r.t.setPowerZero()),
                        Commands.wait(1.0),
                        r.i.in(),
                        Commands.instant(() -> r.t.face(r.getShootTarget(), p.score)),
                        PedroCommands.follow(r.f, p.next()),
                        r.shoot(),
                        PedroCommands.follow(r.f, p.next()),
                        r.intake()
                                .with(
                                        PedroCommands.follow(r.f, p.next())
                                ),
                        Commands.instant(() -> angle = true),
                        PedroCommands.follow(r.f, p.next()),
                        r.i.idle(),
                        PedroCommands.follow(r.f, p.next()),
                        Commands.wait(500.0),
                        PedroCommands.follow(r.f, p.next()),
                        
                        r.shoot(),
                        PedroCommands.follow(r.f, p.next()),
                        r.intake()
                                .with(
                                        PedroCommands.follow(r.f, p.next())
                                ),
                        PedroCommands.follow(r.f, p.next()),
                        
                        r.shoot(),
                        PedroCommands.follow(r.f, p.next()),
                        r.intake()
                                .with(
                                        PedroCommands.follow(r.f, p.next())
                                ),
                        PedroCommands.follow(r.f, p.next()),
                        
                        r.shoot()//,
                       // PedroCommands.follow(r.f, p.next())
                )
        );
    }

    @Override
    public void stop() {
        r.saveEnd();
        reset();
    }
}
