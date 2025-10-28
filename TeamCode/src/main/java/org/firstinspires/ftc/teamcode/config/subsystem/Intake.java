package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;

@Config
@Configurable
public class Intake {
    private final DcMotorEx i;
    public static double idle = 0;
    public static double in = 1;
    public static double out = -1;
    private ColorRangeSensor l;
    private ColorRangeSensor r;
    private ColorRangeSensor d;


    public Intake(HardwareMap hardwareMap) {
        i = hardwareMap.get(DcMotorEx.class, "i");
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

    public void spinIdle() {
        set(idle);
    }

    public Command idle() {
        return new InstantCommand(() -> set(idle));
    }

    public Command in() {
        return new InstantCommand(() -> set(in));
    }

    public Command out() {
        return new InstantCommand(() -> set(out));
    }

    public Command stop() {
        return new InstantCommand(() -> set(0));
    }
}
