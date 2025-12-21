package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.commands.Instant;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config

public class Shooter {
    private DcMotorEx l, r;

    private double t = 0;
    public static double kS = 0.08, kV = 0.00039, kP = 0.001;

    private boolean activated = true;

    public static double near = 1200;
    public static double far = 1400;

    public Shooter(HardwareMap hardwareMap) {
        l = hardwareMap.get(DcMotorEx.class, "sl");
        r = hardwareMap.get(DcMotorEx.class, "sr");
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

    public void shooterToggle() {
        activated = !activated;
        if (!activated)
            setPower(0);
    }

    public Instant toggle() {
        return new Instant(this::shooterToggle);
    }

    public void shootFar() {
        setTarget(far);
        on();
    }

    public void shootNear() {
        setTarget(near);
        on();
    }

    public Instant near() {
        return new Instant(this::shootNear);
    }

    public Instant far() {
        return new Instant(this::shootFar);
    }

    public void setTarget(double velocity) {
        t = velocity;
    }

    public void periodic() {
        if (activated)
            setPower((kV * getTarget()) + (kP * (getTarget() - getVelocity())) + kS);
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

}


