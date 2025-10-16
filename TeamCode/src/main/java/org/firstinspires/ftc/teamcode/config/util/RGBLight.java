package org.firstinspires.ftc.teamcode.config.util;

import com.qualcomm.robotcore.hardware.Servo;

public class RGBLight {

    private Servo s;

    public RGBLight(Servo s) {
        this.s = s;
    }

    public void setPosition(double i) {
        s.setPosition(i);
    }

    public void setColorFromRange(double percent) {
        s.setPosition(percent / 100);
    }

    public double getPosition() {
        return s.getPosition();
    }

    public void red() {
        setPosition(0.279);
    }

    public void orange() {
        setPosition(0.333);
    }

    public void yellow() {
        setPosition(0.388);
    }

    public void sage() {
        setPosition(0.444);
    }

    public void green() {
        setPosition(0.5);
    }

    public void azure() {
        setPosition(0.555);
    }

    public void blue() {
        setPosition(0.611);
    }

    public void indigo() {
        setPosition(0.666);
    }

    public void violet() {
        setPosition(0.722);
    }

    public void off() {
        setPosition(0);
    }

    public void white() {
        setPosition(1);
    }
}
