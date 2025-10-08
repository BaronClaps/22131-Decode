package org.firstinspires.ftc.teamcode.config;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    public final Alliance a;

    public static Pose endPose = new Pose();

    public Robot(HardwareMap h, Alliance a) {
        this.a = a;
        i = new Intake(h);
        l = new Limelight(h, a);
        s = new Shooter(h);
        t = new Turret(h);
        f = Constants.createFollower(h);
    }

    public void periodic() {
        f.update();
        l.periodic();
        s.periodic();
        t.periodic();
    }

    public void stop() {
        endPose = f.getPose();
    }
}
