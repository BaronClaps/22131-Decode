package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Config
public class Turret {
    public static double error = 0, power = 0;
    public static double rpt = 0.008660489741;

    private final DcMotorEx m;
    private PIDFController p; // pidf controller for turret
    public static double t = 0; // target for turret
    public static double kp = .02, kf = 0.0, kd = 0.0004;
    public static double limitDegrees = 220;

    public Turret(HardwareMap hardwareMap) {
        m = hardwareMap.get(DcMotorEx.class, "t");
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        p = new PIDFController(new PIDFCoefficients(kp, 0, kd, kf));
    }

    private void setTurretTarget(double ticks) {
        t = ticks;
    }

    /** ticks */
    public double getTurretTarget() {
        return t;
    }

    /** ticks */
    private void incrementTurretTarget(double ticks) {
        t += ticks;
    }

    public double getTurret() {
        return m.getCurrentPosition();
    }

    public void periodic() {
        p.setCoefficients(new PIDFCoefficients(kp, 0, kd, kf));
        error = getTurretTarget() - getTurret();
        p.updateError(error);
        power = p.run();
        m.setPower(power);
    }

    /** Return yaw in radians */
    public double getYaw() {
        return MathFunctions.normalizeAngle(getTurret() * rpt);
    }

    public void setYaw(double radians) {
        radians = MathFunctions.normalizeAngle(radians);

        if ((radians > Math.toRadians(limitDegrees)) || radians < Math.toRadians(-limitDegrees))
          //  if (true)
           // else
                radians = -((2*Math.PI)-radians);

        setTurretTarget(radians/rpt);
    }

    public void addYaw(double radians) {
        setYaw(getYaw() + radians);
    }

    public void resetTurret() {
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setTurretTarget(0);
    }

    public InstantCommand reset() {
        return new InstantCommand(this::resetTurret);
    }

    public InstantCommand set(double radians) {
        return new InstantCommand(() -> set(radians));

    }

    public InstantCommand add(double radians) {
        return new InstantCommand(() -> setYaw(getYaw() + radians));
    }
}
