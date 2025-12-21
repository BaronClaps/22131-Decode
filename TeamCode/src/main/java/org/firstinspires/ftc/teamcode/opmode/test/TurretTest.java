package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.util.Alliance;

@Config
@TeleOp
public class TurretTest extends OpMode {
    Robot r;

    public static double target = 0;
    public static boolean face = false;
    public static double x = 0, y = 0;
    MultipleTelemetry telemetryM;

    public void init() {
        r = new Robot(hardwareMap, Alliance.BLUE);
        r.d.setStart(new Pose());
        r.t.resetTurret();

        telemetryM = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        if (face)
            r.t.face(new Pose(x, y), r.d.getPose());
        else
            r.t.setYaw(Math.toRadians(target));

        r.periodic();
        telemetryM.addData("error: ", r.t.getError());
        telemetryM.addData("isReady? ", r.t.isReady());
        telemetryM.update();
    }
}
