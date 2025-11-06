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

public class Robot {
    public final Intake i;
    public final Limelight l;
    public final Shooter s;
    public final Turret t;
    public final Follower f;
    public Alliance a;

    private final LynxModule hub;
    private final Timer loop = new Timer();
    // int loops = 0;
    //private double loopTime = 0;

    public static Pose endPose = new Pose(8+24,6.25+24,0);
    public static Pose shootTarget = new Pose(6, 144-6, 0);

    public Robot(HardwareMap h, Alliance a) {
        this.a = a;
        i = new Intake(h);
        l = new Limelight(h, a);
        s = new Shooter(h);
        t = new Turret(h);
        f = Constants.createFollower(h);

        hub = h.getAll(LynxModule.class).get(0);
        hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        loop.resetTimer();
        setShootTarget();

    }

    public void periodic() {
        setShootTarget();

        //loops++;

        if (loop.getElapsedTime() % 5 == 0) {
            hub.clearBulkCache();
            //loopTime = (double) loop.getElapsedTime() / loops;
        }

        f.update();
        t.periodic();
        s.periodic();
    }

    public void stop() {
        endPose = f.getPose();
    }

//    public double getLoopTime() {
//        return loopTime;
//    }

    public void setShootTarget() {
        if (a == Alliance.BLUE && shootTarget.getX() != 6)
            shootTarget = new Pose(6, 144 - 6, 0);
        else if (a == Alliance.RED && shootTarget.getX() != (144 - 6))
            shootTarget = shootTarget.mirror();
    }

    public Pose getShootTarget() {
        return shootTarget;
    }
}
