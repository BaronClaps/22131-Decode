package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;

@Config
@Configurable
public class Intake {
    private final DcMotorEx i;
    public static double idle = 0.1;
    public static double in = 1;
    public static double out = -1;

    public Intake(HardwareMap hardwareMap) {
        i = hardwareMap.get(DcMotorEx.class, "i");
    }

    public void setPower(double power) {
        i.setPower(power);
    }

    public Command idle() {
        return new InstantCommand(() -> setPower(idle));
    }

    public Command in() {
        return new InstantCommand(() -> setPower(in));
    }

    public Command out() {
        return new InstantCommand(() -> setPower(out));
    }

    public Command stop() {
        return new InstantCommand(() -> setPower(0));
    }
}
