package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Config
@Configurable

public class Shooter extends SubsystemBase {
    private Servo f;
    private DcMotorEx l, r;

    private double t = 0;
    public static double kS, kV, kP;

    private boolean activated = true;

    public static double close = 1250;
    public static double far = 1400;
    public static double flipUp = 0.3;
    public static double flipDown = 0.71;

    public Shooter(HardwareMap hardwareMap) {
        l = hardwareMap.get(DcMotorEx.class, "sl");
        r = hardwareMap.get(DcMotorEx.class, "sr");
        f = hardwareMap.get(Servo.class, "f");
        r.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double getTarget() {
        return t;
    }

    public double getVelocity() {
        return l.getVelocity();
    }

    public void setPower(double p) {
        l.setPower(p);
        r.setPower(p);
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

    @Override
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

    public void forDistance(double distance) {
        //setTarget((6.13992 * distance) + 858.51272);
        setTarget((0.00180088*Math.pow(distance, 2))+(4.14265*distance)+948.97358);
    }

    public boolean atUp() {
        return f.getPosition() == flipUp;
    }

}


