package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.subsystem.Turret;

@TeleOp
public class TurretTest extends OpMode {
    public Turret t;

    @Override
    public void init() {
        t = new Turret(hardwareMap);
        t.reset();
    }

    @Override
    public void loop() {
        t.periodic();
    }
}
