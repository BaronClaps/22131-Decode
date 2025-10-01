package org.firstinspires.ftc.teamcode.config.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.config.Robot;

public class AimTurret extends CommandBase {
    private final Robot r;
    private final double rad;
    private final boolean add;

    public AimTurret(Robot robot, double radians, boolean add) {
        this.r = robot;
        this.rad = radians;
        this.add = add;
    }

    public AimTurret(Robot robot, double radians) {
        this(robot, radians, false);
    }

    @Override
    public void initialize() {
        if (add)
         r.t.addYaw(rad);
        else
         r.t.setYaw(rad);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(r.t.getYaw() - rad) < 0.05;
    }

}
