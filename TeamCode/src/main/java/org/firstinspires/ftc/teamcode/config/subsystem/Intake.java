package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Instant;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Intake {
    private final DcMotorEx i;
    public static double off = 0;
    public static double idle = 0.5;
    public static double in = 1;
    public static double out = -1;


    public Intake(HardwareMap hardwareMap) {
        i = hardwareMap.get(DcMotorEx.class, "i");
        set(0);
    }

    public void set(double power) {
        i.setPower(power);
    }

    public void spinIn() {
        set(in);
    }

    public void spinOut() {
        set(out);
    }

    public void spinOff() {
        set(off);
    }

    public void spinIdle() {
        set(idle);
    }

    public Command off() {
        return new Instant(() -> set(off));
    }

    public Command in() {
        return new Instant(() -> set(in));
    }

    public Command out() {
        return new Instant(() -> set(out));
    }

    public Command idle() {
        return new Instant(this::spinIdle);
    }

    public Instant toggleIn() {
        return new Instant(() -> {
            if (i.getPower() != 0)
                spinOff();
            else
                spinIn();
        });
    }

    public Instant toggleOut() {
        return new Instant(() -> {
            if (i.getPower() != 0)
                spinOff();
            else
                spinOut();
        });
    }

    public String getCurrent() {
        return "Intake Motor: " + i.getCurrent(CurrentUnit.AMPS);
    }
}
