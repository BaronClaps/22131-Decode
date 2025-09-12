package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Config
public class Turret {
    public static double zero_yaw = 0;
    public static double limit_yaw = 200;
    public static double error = 0, power = 0;
    public static double rpt = 0.008660489741;

    private DcMotorEx m;
    private PIDFController p; // pidf controller for turret
    public static double t = 0; // target for turret
    public static double kp = 0.01, kf = 0.0, kd = 0.05;

    public Turret(HardwareMap hardwareMap) {
        m = hardwareMap.get(DcMotorEx.class, "t");
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        p = new PIDFController(new PIDFCoefficients(kp, 0, kd, kf));
    }

    private void setTurretTarget(double degrees) {
        t = degrees;
    }

    /** degrees */
    public double getTurretTarget() {
        return t;
    }

    /** degrees */
    private void incrementTurretTarget(double degrees) {
        t += degrees;
        t = MathFunctions.clamp(t, zero_yaw - limit_yaw, zero_yaw + limit_yaw);
    }

    private double getTurret() {
        double angle = m.getCurrentPosition() * rpt;
        return MathFunctions.normalizeAngle(-angle);
    }

    public void periodic() {
        p.setCoefficients(new PIDFCoefficients(kp, 0, kd, kf));
        error = Math.toRadians(getTurretTarget()) - getTurret();
        p.updateError(error);
        power = p.run();
        m.setPower(power);
    }

    /** Return yaw in radians */
    public double getYaw() {
        return getTurret();
    }

    public void setYaw(double degrees) {
        setTurretTarget(degrees);
    }

    public void addYaw(double degrees) {
        incrementTurretTarget(degrees);
    }

    public void reset() {
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setTurretTarget(0);
    }
}
