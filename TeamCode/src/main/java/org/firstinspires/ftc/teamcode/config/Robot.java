package org.firstinspires.ftc.teamcode.config;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.pedro.Constants;
import org.firstinspires.ftc.teamcode.config.subsystem.Intake;
import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystem.Turret;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.vision.Limelight;

import java.util.List;

public class Robot {
    public final Intake i;
    public final Limelight l;
    public final Shooter s;
    public final Turret t;
    public final Follower f;
    public Alliance a;

    private final List<LynxModule> hubs;
    private final Timer loop = new Timer();
    private int loops = 0;
    private double loopTime = 0;

    public static Pose endPose = new Pose(8,6.25,0);

    public Robot(HardwareMap h, Alliance a) {
        this.a = a;
        i = new Intake(h);
        l = new Limelight(h, a);
        s = new Shooter(h);
        t = new Turret(h);
        f = Constants.createFollower(h);

        hubs = h.getAll(LynxModule.class);

        for (LynxModule module : hubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        loop.resetTimer();

    }

    public void periodic() {
        loops++;

        if (loop.getElapsedTime() % 5 == 0) {
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }

            loopTime = (double) loop.getElapsedTime() / loops;
        }

        f.update();
        t.periodic();
        s.periodic();
    }

    public void stop() {
        endPose = f.getPose();
    }

    public double getLoopTime() {
        return loopTime;
    }

}
