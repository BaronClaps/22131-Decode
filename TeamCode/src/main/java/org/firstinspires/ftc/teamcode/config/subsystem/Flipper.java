package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config

public class Flipper {
    private Servo g;
    public static double down = 0.52;
    public static double up = 0.3;

    public Flipper(HardwareMap hardwareMap) {
        g = hardwareMap.get(Servo.class, "f");
    }

    public void flipDown() {
        g.setPosition(down);
    }

    public void flipUp() {
        g.setPosition(up);
    }

    public CommandBuilder up() { return Commands.instant(this::flipUp); }
    public CommandBuilder down() { return Commands.instant(this::flipDown); }

    public void toggle() {
        if (g.getPosition() == up)
            flipDown();
        else
            flipUp();
    }

    public boolean closed() {
        return g.getPosition() == down;
    }

}


