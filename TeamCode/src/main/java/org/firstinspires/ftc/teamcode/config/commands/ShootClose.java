package org.firstinspires.ftc.teamcode.config.commands;

import com.pedropathing.util.Timer;
import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.config.Robot;

public class ShootClose extends CommandBase {
    private final Robot r;
    private int st = 0;
    private Timer t = new Timer();

    public ShootClose(Robot robot) {
        this.r = robot;
    }

    @Override
    public void initialize() {
        setState(0);
    }

    @Override
    public void execute() {
        switch (st) {
            case 0:
                r.s.down();
                r.i.in();
                r.s.close();
                setState(1);
                break;
            case 1:
                if (t.getElapsedTime() > 1000 && !r.f.isBusy()) {
                    r.i.idle();
                    r.s.up();
                    setState(-1);
                }
                break;
        }
    }


    @Override
    public boolean isFinished() {
        return st == -1;
    }

    public void setState(int x) {
        st = x;
        t.resetTimer();
    }

}
