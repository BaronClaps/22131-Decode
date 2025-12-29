package org.firstinspires.ftc.teamcode.config;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Sequential;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.pedro.Constants;
import org.firstinspires.ftc.teamcode.config.subsystem.*;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.vision.Limelight;

public class Robot {
    public final Intake i;
    public final Limelight l;
    public final Shooter s;
    public final Gate g;
    public final Turret t;
    public final Follower f;
    public Alliance a;

    private final LynxModule hub;
    private final Timer loop = new Timer();
    public static Pose defaultPose = new Pose(8 + 24, 6.25 + 24, 0);
    public static Pose shootTarget = new Pose(6, 144 - 6, 0);

    public Robot(HardwareMap h, Alliance a) {
        this.a = a;
        i = new Intake(h);
        l = new Limelight(h, a);
        s = new Shooter(h);
        g = new Gate(h);
        t = new Turret(h);
        f = Constants.createFollower(h);
        f.setStartingPose(a == Alliance.RED ? defaultPose : defaultPose.mirror());

        hub = h.getAll(LynxModule.class).get(0);
        hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        loop.resetTimer();
        setShootTarget();

        periodic();
    }

    public void periodic() {
        setShootTarget();

        if (loop.getElapsedTime() % 5 == 0) {
            hub.clearBulkCache();
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

    public Sequential shoot() {
        return new Sequential(
                g.close(),
                s.near(),
                new WaitUntil(s::atTarget),
                i.in(),
                g.open(),
                new Wait(800),
                i.off(),
                new Wait(200),
                i.in(),
                new Wait(400),
                g.close()
        );
    }

    public Sequential intake() {
        return new Sequential(
                g.open(),
                i.in(),
                new Wait(500)
        );
    }
}
