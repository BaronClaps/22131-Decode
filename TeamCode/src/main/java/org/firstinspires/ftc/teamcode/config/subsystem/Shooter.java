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
    public static double zero_yaw = 0;
    public static double limit_yaw = 0.55;
    public static double zero_pitch = 0.1;
    public static double limit_pitch = 0.65;
    public static double range_pitch = 20; // degrees between limits

    private static final double g = -386.0885; // in/s^2

    private MotorEx m_turret, m_spinner;
    private AbsoluteEncoder e_turret; // absolute encoder for turret
    private PIDFController p_turret; // pidf controller for turret
    private ServoEx s_hood; // servo for hood
    private double t_turret; // target for turret

    public Shooter(HardwareMap hardwareMap) {
        m_turret = new MotorEx(hardwareMap, "t", Motor.GoBILDA.RPM_435);
        e_turret = new AbsoluteEncoder(hardwareMap.analogInput.get("te"), 3.3)
                .zero(zero_yaw);
        p_turret = new PIDFController(new PIDFCoefficients(.03, 0, 0.01, 0));
        // m_turret.motorEx.getVelocity(AngleUnit.RADIANS);
        s_hood = new ServoEx(hardwareMap, "h");
        m_spinner = new MotorEx(hardwareMap, "w", Motor.GoBILDA.BARE);
    }

    private void setTurretTarget(double degrees) {
        double upper_limit = zero_yaw + limit_yaw * 360;
        double lower_limit = zero_yaw - limit_yaw * 360;
        if (degrees < lower_limit) degrees = lower_limit;
        if (degrees > upper_limit) degrees = upper_limit;
        t_turret = degrees;
    }

    /** degrees */
    private double getTurretTarget() {
        return t_turret;
    }

    /** degrees */
    private void incrementTurretTarget(double degrees) {
        t_turret += degrees;
        double upper_limit = zero_yaw + limit_yaw * 360;
        double lower_limit = zero_yaw - limit_yaw * 360;
        if (t_turret < lower_limit) t_turret = lower_limit;
        if (t_turret > upper_limit) t_turret = upper_limit;
    }

    /** degrees */
    private double getTurret() {
        double angle = (e_turret.getVoltage() - zero_yaw) / 3.3 * 360;
        if (angle < 0) angle += 360;
        if (angle > 180) angle -= 360;
        return angle;
    }

    private void periodic_turret() {
        double error = t_turret - getTurret();
        p_turret.updateError(error);
        double power = p_turret.run();
        m_turret.set(power);
    }

    private void setHood(double pos) {
        s_hood.set(pos);
    }

    /** degrees */
    private double getHood() {
        return s_hood.get();
    }

    public void setPitch(double degrees) {
        double pos = (degrees / range_pitch) + zero_pitch;
        if (pos < 0) pos = 0;
        if (pos > limit_pitch) pos = limit_pitch;
        s_hood.set(pos);
    }

    public void addPitch(double degrees) {
        setPitch((getHood() - zero_pitch) * range_pitch + degrees);
    }

    public double getPitch() {
        return (getHood() - zero_pitch) * range_pitch;
    }

    public double getYaw() {
        return getTurret();
    }

    public void setYaw(double degrees) {
        setTurretTarget(degrees);
    }

    public void addYaw(double degrees) {
        incrementTurretTarget(degrees);
    }

    /**
     * Solve for possible launch angles theta (radians).
     Only returns positive angles (0 < theta < pi/2).*
     @param v initial speed
     @param h initial height
     @param x horizontal distance to target
     @param y target height
     @return array of positive solutions (length 0, 1, or 2)*/
    public static double[] solveTheta(double v, double h, double x, double y) {
        if (x == 0) {
            return new double[0]; // avoid division by zero}
        }
        // coefficients
        double A = (g * x * x) / (2 * v * v);
        double C = h - y + A;

        // discriminant
        double disc = x * x - 4 * A * C;
        if (disc < 0) {
            return new double[0]; // no real solution
        }

        double sqrtDisc = Math.sqrt(disc);

        double T1 = (-x + sqrtDisc) / (2 * A);
        double T2 = (-x - sqrtDisc) / (2 * A);

        // convert to angles and filter only positive
        java.util.List<Double> solutions = new java.util.ArrayList<>();

        if (disc == 0) {
            double theta = Math.atan(T1);
            if (theta > 0) solutions.add(theta);
        } else {
            double theta1 = Math.atan(T1);
            double theta2 = Math.atan(T2);

            if (theta1 > 0) solutions.add(theta1);
            if (theta2 > 0) solutions.add(theta2);
        }

        // convert list to array
        double[] result = new double[solutions.size()];
        for (int i = 0; i < solutions.size(); i++) {
            result[i] = solutions.get(i);
        }

        return result;
    }

    public void aimTheta(double distance){
        double v = 267; // initial speed in in/s (recalc says estimated 22.25 ft/s)
        double h = 12; // initial height in in
        double y = 38; // target height in in
        double[] radians = solveTheta(v, h, distance, y);
        double[] degrees = new double[radians.length];
        for (int i = 0; i < radians.length; i++) {
            degrees[i] = Math.toDegrees(radians[i]);
        }
        double theta = 0;
        for (double degree : degrees) {
            if (degree > 0 && degree < range_pitch) {
                theta = degree;
                break;
            }
        }

        setPitch(theta);
    }


    @Override
    public void periodic() {
        periodic_turret();
    }
}


