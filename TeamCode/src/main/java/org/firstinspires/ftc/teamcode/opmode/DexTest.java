package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="No Indexation without Representation!", group="Tests")
public class DexTest extends OpMode {
    Servo l, r;

    public static double perIndex = 0.166666667;
    @Override
    public void init() {
        l = hardwareMap.get(Servo.class, "ld");
        r = hardwareMap.get(Servo.class, "rd");

        l.setPosition(0.5);
        r.setPosition(0.5);
    }

    @Override
    public void loop() {
        if (gamepad1.rightBumperWasPressed()) {
            l.setPosition(l.getPosition() + perIndex);
            r.setPosition(r.getPosition() + perIndex);
        }

        if (gamepad1.leftBumperWasPressed()) {
            l.setPosition(l.getPosition() - perIndex);
            r.setPosition(r.getPosition() - perIndex);
        }

        if (gamepad1.aWasPressed()) {
            l.setPosition(0.5);
            r.setPosition(0.5);
        }

        if (gamepad1.bWasPressed()) {
            l.setPosition(1);
            r.setPosition(1);
        }

        if (gamepad1.aWasPressed()) {
            l.setPosition(0);
            r.setPosition(0);
        }
    }
}
