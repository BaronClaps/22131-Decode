package org.firstinspires.ftc.teamcode.config.subsystem;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.util.AbsoluteEncoder;

import java.util.Arrays;

public class Shooter extends SubsystemBase {
    private Servo f;
    private MotorEx l, r;
    private PIDFController p;
    private double t;
    public static double kp = 0.03, kd = 0.01;
    private boolean activated = false;

    public static double flipUp = 0.5;
    public static double flipDown = 0.3;

    public Shooter(HardwareMap hardwareMap) {
        p = new PIDFController(new PIDFCoefficients(kp, 0, kd, 0));
        l = hardwareMap.get(MotorEx.class, "sl");
        r = hardwareMap.get(MotorEx.class, "sr");
        f = hardwareMap.get(Servo.class, "f");
        r.setInverted(true);
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
        l.set(p);
        r.set(p);
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

    public void setTarget(double velocity) {
        t = velocity;
    }

    @Override
    public void periodic() {
        p.setCoefficients(new PIDFCoefficients(kp, 0, kd, 0));

        if (activated) {
            p.updateError(getTarget() - getVelocity());
            setPower(p.run());
        }
    }

    public void up() {
        f.setPosition(flipUp);
    }

    public void down() {
        f.setPosition(flipDown);
    }

}


