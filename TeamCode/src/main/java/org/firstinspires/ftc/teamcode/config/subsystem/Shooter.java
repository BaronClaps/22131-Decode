package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.commands.Instant;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import smile.interpolation.BilinearInterpolation;
import smile.interpolation.Interpolation2D;

@Config

public class Shooter {
    private DcMotorEx l, r;

    private double t = 0;
    public static double kS = 0.08, kV = 0.00039, kP = 0.01;

    private boolean activated = true;

    public static double near = 1200;
    public static double far = 1400;

    private static final double[] xs = {44, 72, 100};
    private static final double[] ys = {10, 38, 66};
    private static final double[][] closeVelocities = {
            {1200, 1200, 1275},
            {1275, 1275, 1350},
            {1325, 1360, 1400}
    };
    public static final Interpolation2D closeInterpolation = new BilinearInterpolation(xs, ys, closeVelocities);
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
    public void forPose(Pose current, Pose target, boolean close) {
        double xDistance = Math.abs(target.getX()-current.getX());
        double yDistance = Math.abs(target.getY()-current.getY());

        if (close)
            setTarget(closeInterpolation.interpolate(xDistance, yDistance));
    }

    public String getLeftCurrent() {
        return "Left Shooter Motor: " + l.getCurrent(CurrentUnit.AMPS);
    }

    public String getRightCurrent() {
        return "Right Shooter Motor: " + r.getCurrent(CurrentUnit.AMPS);
    }

}


