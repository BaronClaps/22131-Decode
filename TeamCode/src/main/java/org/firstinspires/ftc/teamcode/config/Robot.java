package org.firstinspires.ftc.teamcode.config;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.pedro.Constants;
import org.firstinspires.ftc.teamcode.config.subsystem.*;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.vision.Limelight;

import java.util.List;

import static com.pedropathing.ivy.groups.Groups.sequential;

public class Robot {
    public final Intake i;
    public final Limelight l;
    public final Shooter s;
    public final Flipper g;
    public final Turret t;
    public final Follower f;
    public Alliance a;

    private final List<LynxModule> hubs;
    private final Timer loop = new Timer();
    public double loops = 0, lastLoop = 0, loopTime = 0;
    public static Pose defaultPose = new Pose(8 + 24, 6.25 + 24, 0);
    public static Pose shootTarget = new Pose(6, 144 - 6, 0);

    public Robot(HardwareMap h, Alliance a) {
        this.a = a;
        i = new Intake(h);
        l = new Limelight(h, a);
        s = new Shooter(h);
        g = new Flipper(h);
        t = new Turret(h);
        f = Constants.createFollower(h);

        hubs = h.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        loop.resetTimer();
        setShootTarget();

        periodic();
    }

    public void periodic() {
        setShootTarget();

//        if (loop.getElapsedTime() % 10 == 0) {
//            hub.clearBulkCache();
//        }

        loops++;

        if (loops > 10) {
            double now = loop.getElapsedTime();
            loopTime = (now - lastLoop) / loops;
            lastLoop = now;
            loops = 0;
        }

        //d.periodic();
        f.update();
        t.periodic();
        s.periodic();
    }

    public void saveEnd() {
        defaultPose = f.getPose();
    }


    public void setShootTarget() {
        if (a == Alliance.BLUE && shootTarget.getX() != 6)
            shootTarget = new Pose(6, 144 - 6, 0);
        else if (a == Alliance.RED && shootTarget.getX() != (144 - 6))
            shootTarget = shootTarget.mirror();
    }

    public Pose getShootTarget() {
        return shootTarget;
    }

    public CommandBuilder shoot() {
        return sequential(
                g.down(),
                i.in(),
                Commands.waitUntil(s::atTarget),
                i.in(),
                Commands.wait(200.0),
                g.down(),
                Commands.wait(400.0),
                g.up(),
                Commands.wait(200.0),
                g.down(),
                //i.off(),
                Commands.wait(300.0),
                i.in(),
                Commands.wait(300.0),
                g.up(),
                Commands.wait(200.0),
                g.down(),
                //i.off(),
                Commands.wait(300.0),
                i.in(),
                Commands.wait(300.0),
                g.up(),
                Commands.wait(200.0),
                g.down(),
                i.out()
        );
    }

    public CommandBuilder intake() {
        return sequential(
                g.down(),
                i.in(),
                Commands.wait(500.0)
        );
    }

    public String getCurrent() {
        return i.getCurrent() + "/n" + s.getLeftCurrent() + "/n" + s.getRightCurrent() + "/n" + t.getCurrent();
    }

    public double getLoopTimeMs() {
        return loopTime;
    }

    public double getLoopTimeHz() {
        return 1000 / loopTime;
    }
}
