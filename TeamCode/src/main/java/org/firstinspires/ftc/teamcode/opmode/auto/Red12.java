package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.*;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.commands.FollowPath;
import org.firstinspires.ftc.teamcode.config.commands.IntakeIn;
import org.firstinspires.ftc.teamcode.config.commands.Shoot;
import org.firstinspires.ftc.teamcode.config.paths.NoCorner12;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.util.OpModeCommand;

@Autonomous(name = "Red 12", group = "Interesting")
public class Red12 extends OpModeCommand {
    Robot r;

    @Override
    public void initialize() {
        r = new Robot(hardwareMap, Alliance.RED);
        NoCorner12 p = new NoCorner12(r);
        r.f.setStartingPose(p.start);

        r.s.down();
        r.t.resetTurret();

        schedule(
                new RunCommand(r::periodic),
                //new RunCommand(() -> r.t.face(r.getShootTarget(), r.f.getPose())),
                new RunCommand(() -> {
                    double dist = r.getShootTarget().distanceFrom(r.f.getPose());
                    r.s.forDistance(dist);
                }),
                new RunCommand(() -> {
                    telemetry.addData("Pose: ", r.f.getPose());
                    telemetry.addData("Follower Busy: ", r.f.isBusy());
                    telemetry.addData("Shooter At Target: ", r.s.atTarget());
                    telemetry.addData("Flipper at Up: ", r.s.atUp());
                    telemetry.addData("Turret Ticks: ", r.t.getTurret());
                    telemetry.update();
                }),
                new SequentialCommandGroup(
                        new WaitCommand(1),
                        r.i.in(),
                        new InstantCommand(() -> r.t.face(r.getShootTarget(), p.score)),
                        new FollowPath(r, p.next())
                                .alongWith(
                                        new WaitUntilCommand(() -> r.f.getCurrentTValue() >= 0.25)
                                                .andThen(
                                                        new InstantCommand(() -> r.s.on())
                                                )
                                ),
                        new Shoot(r),
                        new IntakeIn(r)
                                .alongWith(
                                        new FollowPath(r, p.next())
                                ),
                        new FollowPath(r, p.next())
                                .alongWith(
                                        new WaitUntilCommand(() -> r.f.getCurrentTValue() >= 0.25)
                                                .andThen(
                                                        new InstantCommand(() -> r.s.on())
                                                )
                                ),
                        new Shoot(r),
                        new IntakeIn(r)
                                .alongWith(
                                        new FollowPath(r, p.next())
                                ),
                        new FollowPath(r, p.next()),
                        new FollowPath(r, p.next())
                                .alongWith(
                                        new WaitUntilCommand(() -> r.f.getCurrentTValue() >= 0.25)
                                                .andThen(
                                                        new InstantCommand(() -> r.s.on())
                                                )
                                ),
                        new Shoot(r),
                        new IntakeIn(r)
                                .alongWith(new FollowPath(r, p.next())),
                        new InstantCommand(() -> r.t.face(r.getShootTarget(), p.scoreCorner)),
                        new FollowPath(r, p.next())
                                .alongWith(
                                        new WaitUntilCommand(() -> r.f.getCurrentTValue() >= 0.25)
                                                .andThen(
                                                        new InstantCommand(() -> r.s.on())
                                                )
                                ),
                        new Shoot(r),
                        new IntakeIn(r)
                                .alongWith(new FollowPath(r, p.next())),
                        new FollowPath(r, p.next())
                                .alongWith(
                                        new WaitUntilCommand(() -> r.f.getCurrentTValue() >= 0.25)
                                                .andThen(
                                                        new InstantCommand(() -> r.s.on())
                                                )
                                ),
                        new Shoot(r)
                )
        );
    }

    @Override
    public void stop() {
        r.stop();
    }
}
