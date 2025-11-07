package org.firstinspires.ftc.teamcode.opmode.auto;

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

@Autonomous(name = "Blue 12", group = "Interesting")
public class Blue12 extends OpModeCommand {
    Robot r;

    @Override
    public void initialize() {
        r = new Robot(hardwareMap, Alliance.BLUE);
        NoCorner12 p = new NoCorner12(r);
        r.f.setStartingPose(p.start);

        r.s.down();
        r.t.resetTurret();

        schedule(
                new RunCommand(r::periodic),
                new RunCommand(() -> r.t.face(r.getShootTarget(), r.f.getPose())),
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
                        new FollowPath(r, p.next())
                                .andThen(
                                        new WaitCommand(500),
                                        new ShootClose(r),
                                        new ShootClose(r),
                                        new ShootClose(r)
                                ),
                        new IntakeIn(r)
                                .alongWith(
                                        new FollowPath(r, p.next())
                                ),
                        new FollowPath(r, p.next()),
                        new WaitCommand(1500),
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
                                ),
                        new FollowPath(r, p.next())
                )
        );
    }

    @Override
    public void stop() {
        r.stop();
    }
}
