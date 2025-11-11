package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.util.Alliance;

@TeleOp
public class RapidTest extends OpMode {
    Robot r;
    private Timer timer = new Timer();
    @Override
    public void init() {
        r = new Robot(hardwareMap, Alliance.BLUE);
        timer = new Timer();
    }

    public void start() {
        timer.resetTimer();
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            r.s.close();
            r.i.spinIn();
        }

        if (gamepad1.bWasPressed()) {
            r.s.off();
            r.i.spinIdle();
        }

        if (gamepad1.yWasPressed())
            timer.resetTimer();

        if (timer.getElapsedTimeSeconds() > 1.25)
            r.s.down();
        else if (timer.getElapsedTimeSeconds() > 1)
            r.s.up();
        else if (timer.getElapsedTimeSeconds() > .75)
            r.s.down();
        else if (timer.getElapsedTimeSeconds() > .5)
            r.s.up();
        else if (timer.getElapsedTimeSeconds() > 0.25)
            r.s.down();
        else if (timer.getElapsedTimeSeconds() > 0)
            r.s.up();

        r.periodic();
    }
}
