package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Turret {
    public static double error = 0, power = 0, manualPower = 0;
    public static double rpt = 0.0029919;

    public final DcMotorEx m;
    private PIDFController p, s; // pidf controller for turret
    public static double t = 0;
    public static double pidfSwitch = 30; // target for turret
    public static double kp = 0.003, kf = 0.0, kd = 0.000, sp = .005, sf = 0, sd = 0.0001;

    public static boolean on = true, manual = false;

    public Turret(HardwareMap hardwareMap) {
        m = hardwareMap.get(DcMotorEx.class, "t");
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m.setPower(0);

        p = new PIDFController(new PIDFCoefficients(kp, 0, kd, kf));
        s = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));
    }

    private void setTurretTarget(double ticks) {
        t = ticks;
    }

    public double getTurretTarget() {
        return t;
    }

    public double getTurret() {
        return m.getCurrentPosition();
    }

    public void periodic() {
        if (on) {
            if (manual) {
                m.setPower(manualPower);
                return;
            }
            p.setCoefficients(new PIDFCoefficients(kp, 0, kd, kf));
            s.setCoefficients(new PIDFCoefficients(sp, 0, sd, sf));
            error = getTurretTarget() - getTurret();
            if (Math.abs(error) > pidfSwitch) {
                p.updateError(error);
                p.updateFeedForwardInput(Math.signum(error));
                power = p.run();
            } else {
                s.updateError(error);
                power = s.run();
            }

            m.setPower(power);
        } else {
            m.setPower(0);
        }
    }

    public void manual(double power) {
        manual = true;
        manualPower = power;
    }

    public void automatic() {
        manual = false;
    }

    public void on() {
        on = true;
    }

    public void off() {
        on = false;
    }

    public double getYaw() {
        return normalizeAngle(getTurret() * rpt);
    }

    public void setYaw(double radians) {
        radians = normalizeAngle(radians);
        setTurretTarget(radians/rpt);
    }

    public void addYaw(double radians) {
        setYaw(getYaw() + radians);
    }

    public void face(Pose targetPose, Pose robotPose) {
        double angleToTargetFromCenter = Math.atan2(targetPose.getY() - robotPose.getY(), targetPose.getX() - robotPose.getX());
        double robotAngleDiff = normalizeAngle(angleToTargetFromCenter - robotPose.getHeading());
        setYaw(robotAngleDiff);
    }

    public void resetTurret() {
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setTurretTarget(0);
    }

    public CommandBuilder reset() {
        return Commands.instant(this::resetTurret);
    }

    public CommandBuilder set(double radians) {
        return Commands.instant(() -> setYaw(radians));

    }

    public CommandBuilder add(double radians) {
        return Commands.instant(() -> setYaw(getYaw() + radians));
    }

    public static double normalizeAngle(double angleRadians) {
        double angle = angleRadians % (Math.PI * 2D);
        if (angle <= -Math.PI) angle += Math.PI * 2D;
        if (angle > Math.PI) angle -= Math.PI * 2D;
        return angle;
    }

    public double getError() {
        return error;
    }

    public boolean isReady() {
        return Math.abs(getError()) < 30;
    }

    public CommandBuilder left(boolean on) {
        return Commands.instant(() -> {
            if (on)
                manual(-0.5);
        });
    }

    public CommandBuilder right(boolean on) {
        return Commands.instant(() -> {
            if (on)
                manual(0.5);
        });
    }

    public String getCurrent() {
        return "Turret Motor: " + m.getCurrent(CurrentUnit.AMPS);
    }
}
