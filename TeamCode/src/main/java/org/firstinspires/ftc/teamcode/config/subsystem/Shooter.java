package org.firstinspires.ftc.teamcode.config.subsystem;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.util.AbsoluteEncoder;

import java.util.Arrays;

public class Shooter extends SubsystemBase {
    private static final double g = -386.0885; // in/s^2

    private MotorEx l, r;
    private PIDFController p;
    private double t;
    public static double kp = 0.03, kd = 0.01;
    private boolean activated = false;
    public static double max_v = ((2 * Math.PI)/28);

    public Shooter(HardwareMap hardwareMap) {
        p = new PIDFController(new PIDFCoefficients(kp, 0, kd, 0));
        l = hardwareMap.get(MotorEx.class, "sl");
        r = hardwareMap.get(MotorEx.class, "sr");
        r.setInverted(true);
    }

    /** in/s */
    public double getTargetVelocity() {
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

    public void setTargetVelocity(double velocity) {
        t = velocity;
    }

    @Override
    public void periodic() {
        p.setCoefficients(new PIDFCoefficients(kp, 0, kd, 0));

        if (activated) {
            p.updateError(getTargetVelocity() - getVelocity());
            setPower(p.run());
        }
    }

}


