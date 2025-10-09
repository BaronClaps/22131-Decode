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
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.util.AbsoluteEncoder;

import java.util.Arrays;
// @Config
@Configurable

public class Shooter extends SubsystemBase {
    private Servo f;
    private DcMotorEx l, r;
    private PIDFController b, s;

    private double t = 0;
    public static double bp = 0.03, bd = 0.0, bf = 0.0, sp = 0.01, sd = 0.0001, sf = 0.0;

    public static double pSwitch = 50;
    private boolean activated = true;

    public static double close = 1200;
    public static double far = 2000;
    public static double flipUp = 0.3;
    public static double flipDown = 0.5;

    public Shooter(HardwareMap hardwareMap) {
        b = new PIDFController(new PIDFCoefficients(bp, 0, bd, bf));
        s = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));
        l = hardwareMap.get(DcMotorEx.class, "sl");
        r = hardwareMap.get(DcMotorEx.class, "sr");
        f = hardwareMap.get(Servo.class, "f");
        r.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /** in/s */
    public double getTarget() {
        return t;
    }

    /** in/s */
    public double getVelocity() {
        return l.getVelocity();
    }

    public void setPower(double p) {
        l.setPower(p);
        r.setPower(p);
    }

    public void toggle() {
        activated = !activated;
        if (!activated) setPower(0);
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
        b.setCoefficients(new PIDFCoefficients(bp, 0, bd, bf));
        s.setCoefficients(new PIDFCoefficients(sp, 0, sd, sf));

        if (activated) {
            if (Math.abs(getTarget() - getVelocity()) < pSwitch) {
                s.updateError(getTarget() - getVelocity());
                setPower(s.run());
            } else {
                b.updateError(getTarget() - getVelocity());
                setPower(b.run());
            }
        }
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
        return Math.abs((getTarget()- getVelocity())) < 100;
    }

}


