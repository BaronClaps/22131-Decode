package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.subsystem.Turret;

@TeleOp
@Config
public class TurretTest extends OpMode {
    public Turret t;
    private MultipleTelemetry telemetryM;
    public static double yawDegrees = 0;

    @Override
    public void init() {
        t = new Turret(hardwareMap);
        t.reset();

        telemetryM = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
    }

    @Override
    public void loop() {
        t.setYaw(Math.toRadians(yawDegrees));
        t.periodic();
        telemetryM.addData("target", t.getTurretTarget());
        telemetryM.addData("position", t.getTurret());
        telemetryM.addData("getYaw", t.getYaw());
        telemetryM.addData("error", Turret.error);
        telemetryM.addData("power", Turret.power);
        telemetryM.update();
    }
}
