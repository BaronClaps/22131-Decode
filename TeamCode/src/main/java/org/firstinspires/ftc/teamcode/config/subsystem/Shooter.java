package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config

public class Shooter {
    private Servo f;
    private DcMotorEx l, r;

    private double t = 0;
    public static double kS = 0.08, kV = 0.00039, kP = 0.001;

    private boolean activated = true;

    public static double close = 1200;
    public static double far = 1400;
    public static double flipUp = 0.45;
    public static double flipDown = 0.73;

    public Shooter(HardwareMap hardwareMap) {
        l = hardwareMap.get(DcMotorEx.class, "sl");
        r = hardwareMap.get(DcMotorEx.class, "sr");
        f = hardwareMap.get(Servo.class, "f");
        l.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double getTarget() {
        return t;
    }

    public double getVelocity() {
        return r.getVelocity();
    }

    public void setPower(double p) {
        l.setPower(-p);
        r.setPower(-p);
    }

    public void off() {
        activated = false;
        setPower(0);
    }

    public void on() {
        activated = true;
    }

    public boolean isActivated() {
        return activated;
    }

    public void far() {
        setTarget(far);
        on();
    }

    public void close() {
        setTarget(close);
        on();
    }

    public void setTarget(double velocity) {
        t = velocity;
    }

    public void periodic() {
        if (activated)
            setPower((kV * getTarget()) + (kP * (getTarget() - getVelocity())) + kS);
    }

    public void up() {
        f.setPosition(flipUp);
    }

    public void down() {
        f.setPosition(flipDown);
    }

    public void flip() {
        if (f.getPosition() == flipDown)
            up();
        else
            down();
    }

    public boolean atTarget() {
        return Math.abs((getTarget()- getVelocity())) < 50;
    }

    public void forDistance(double distance, boolean close) {
        //setTarget((6.13992 * distance) + 858.51272);
        if (close)
            //setTarget((0.000367066*Math.pow(distance,4))-(0.124995*Math.pow(distance,3))+(15.63311*Math.pow(distance,2))-(847.49369*distance)+18000.2274);
            setTarget((0.00180088*Math.pow(distance, 2))+(4.14265*distance)+948.97358);
        else
            setTarget(1550);
    }

    public boolean atUp() {
        return f.getPosition() == flipUp;
    }

}


