package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.commands.Instant;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config

public class Gate {
    private Servo g;
    public static double closed = 0.6;
    public static double open = 0.3;

    public Gate(HardwareMap hardwareMap) {
        g = hardwareMap.get(Servo.class, "f");
    }

    public void closeGate() {
        g.setPosition(closed);
    }

    public void openGate() {
        g.setPosition(open);
    }

    public Instant open() { return new Instant(this::openGate); }
    public Instant close() { return new Instant(this::closeGate); }

    public void toggle() {
        if (g.getPosition() == open)
            closeGate();
        else
            openGate();
    }

    public boolean closed() {
        return g.getPosition() == closed;
    }

}


