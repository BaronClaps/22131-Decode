package org.firstinspires.ftc.teamcode.config.util;

import com.qualcomm.robotcore.hardware.Servo;

public class Headlight {

    private Servo s;

    public Headlight(Servo s) {
        this.s = s;
    }

    public void setPosition(double i) {
        s.setPosition(i);
    }

    public void setIntensity(double percent) {
        s.setPosition(percent / 100);
    }

    public double getPosition() {
        return s.getPosition();
    }

    public void max() {
        setPosition(1);
    }

    public void off() {
        setPosition(0);
    }

    public void half() {
        setPosition(0.5);
    }
}
