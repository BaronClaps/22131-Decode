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
import org.firstinspires.ftc.teamcode.config.util.Headlight;
import org.firstinspires.ftc.teamcode.config.util.RGBLight;

@Config
@Configurable

public class Shooter extends SubsystemBase {
    private Servo f;
    private DcMotorEx l, r;
    private PIDFController b, s;
 //   private RGBLight sl;
 //   private Headlight fl;

    private double t = 0;
    public static double bp = 0.03, bd = 0.0, bf = 0.0, sp = 0.01, sd = 0.0001, sf = 0.0;

    public static double pSwitch = 50;
    private boolean activated = true, targetSpotted = false;

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

      //  sl = new RGBLight(hardwareMap.get(Servo.class, "ls"));
      //  fl = new Headlight(hardwareMap.get(Servo.class, "lf"));
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

            // if (atTarget()) {
             //   if (targetSpotted)
                  //  sl.green();
             //   else
                  //  sl.blue();
            //} else {
              //  sl.red();
            //}
        }// else {
         //  // sl.orange();
       // }
    }

    public void up() {
        f.setPosition(flipUp);
   //     fl.max();
    }

    public void down() {
        f.setPosition(flipDown);
      //  fl.off();
    }

    public void flip() {
        if (f.getPosition() == flipDown)
            up();
        else
            down();
    }

    public void targetSpotted(boolean spotted) {
        targetSpotted = spotted;
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


