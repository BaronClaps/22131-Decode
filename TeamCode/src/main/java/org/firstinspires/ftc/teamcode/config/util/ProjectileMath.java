package org.firstinspires.ftc.teamcode.config.util;

import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.util.Range;

public class ProjectileMath {
    private static final double g = -386.0885; // gravity in in/s^2
    private static final double rho = 0.0023769; // air density in slugs/in^3 (~1.225 kg/m^3 converted)
    private static final double CD = 0.4; // drag coefficient, adjust for whiffle ball
    private static final double radius = 5; // ball radius in inches
    private static final double A = Math.PI * radius * radius; // cross-sectional area in in^2
    private static final double massBall = 0.057152639 * 0.0685218; // ball mass in slugs (0.05 kg -> slugs), 1 kg ≈ 0.0685 slugs
    private static final double k = rho * CD * A / (2 * massBall); //Units: 1/in

    /**
     * Computes the coordinates of the ball after being launched with approximate quadratic drag
     * @param time seconds following launch
     * @param v_0 initial launch velocity in in/s
     * @param theta_0 initial launch angle in degrees
     * @param h_0 initial height in inches
     * @return coordinates (in inches)
     */
    public static Vector computePosition(double time, double v_0, double theta_0, double h_0) {
        double convertedTheta = Math.toRadians(theta_0);

        // Horizontal motion with quadratic drag
        double x = 1 / k * Math.log(1 + k * v_0 * Math.cos(convertedTheta) * time);

        // Vertical motion with approximate quadratic drag
        double vy0 = v_0 * Math.sin(convertedTheta);
        double y = h_0 + (vy0 / k) * Math.log(1 + k * vy0 * time) - 0.5 * g * time * time;

        Vector v = new Vector();
        v.setOrthogonalComponents(x, y);
        return v;
    }

    public static Vector getVelocity(double t, double v_0, double theta_0) {
        double v_ox = v_0 * Math.cos(theta_0);
        double v_x = v_ox / (1 + k * v_ox * t);
        double v_oy = v_0 * Math.sin(theta_0);
        double v_y = g * t + v_oy;
        return new Vector(v_x, v_y);
    }

    public static double getConfidence(Vector distances, double v_0, double theta_0) {
        double t = invertX(distances.getXComponent(), v_0, theta_0);
        Vector velocity = getVelocity(t, v_0, theta_0);
        double theta = velocity.getTheta();
        double invertedCost = Math.cos(Math.toRadians(3) - theta);
        return Range.scale(invertedCost, Math.cos(Math.toRadians(18)), 1.0, 0.25, 0.95);
    }

    public static double invertX(double xPos, double v_0, double theta_0) {
        double v_0x = v_0 * Math.cos(theta_0);
        double numerator = Math.exp(k * xPos) - 1;
        double denominator = k * v_0x;
        return numerator / denominator;
    }
}


