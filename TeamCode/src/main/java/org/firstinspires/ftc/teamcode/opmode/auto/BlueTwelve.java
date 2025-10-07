package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.commands.FollowPath;
import org.firstinspires.ftc.teamcode.config.commands.IntakeIn;
import org.firstinspires.ftc.teamcode.config.commands.ShootClose;
import org.firstinspires.ftc.teamcode.config.paths.Artifact12;
import org.firstinspires.ftc.teamcode.config.util.Alliance;

@Autonomous(name = "Blue 12", group = "Interesting")
public class BlueTwelve extends CommandOpMode {
    Robot r;

    @Override
    public void initialize() {
        r = new Robot(hardwareMap, Alliance.BLUE);
        Artifact12 p = new Artifact12(r);
        r.f.setStartingPose(p.start);
        r.s.down();

        schedule(
                new RunCommand(r::periodic),
                new SequentialCommandGroup(
                        new ShootClose(r)
                                .alongWith(
                                        new FollowPath(r, p.next())
                                ),
                        new IntakeIn(r)
                                .alongWith(
                                        new FollowPath(r, p.next())
                                ),
                        new ShootClose(r)
                                .alongWith(
                                        new FollowPath(r, p.next())
                                ),
                        new IntakeIn(r)
                                .alongWith(
                                        new FollowPath(r, p.next())
                                ),
                        new ShootClose(r)
                                .alongWith(
                                        new FollowPath(r, p.next())
                                ),
                        new FollowPath(r, p.next()),
                        new IntakeIn(r)
                                .alongWith(
                                        new FollowPath(r, p.next())
                                ),
                        new ShootClose(r)
                                .alongWith(
                                        new FollowPath(r, p.next())
                                ),
                        new IntakeIn(r)
                                .alongWith(
                                        new FollowPath(r, p.next())
                                ),
                        new ShootClose(r)
                                .alongWith(
                                        new FollowPath(r, p.next())
                                ),
                        new IntakeIn(r)
                                .alongWith(
                                        new FollowPath(r, p.next())
                                ),
                        new ShootClose(r)
                                .alongWith(
                                        new FollowPath(r, p.next())
                                ),
                        new IntakeIn(r)
                                .alongWith(
                                        new FollowPath(r, p.next())
                                ),
                        new ShootClose(r)
                                .alongWith(
                                        new FollowPath(r, p.next())
                                )
                )
        );
    }

    @Override
    public void end() {
        r.stop();
    }
}
