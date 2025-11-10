package org.firstinspires.ftc.teamcode.config.commands;

import com.pedropathing.util.Timer;
import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.config.Robot;

public class Shoot extends CommandBase {
    private final Robot r;
    private int st = 0;
    private Timer t = new Timer();

    public Shoot(Robot robot) {
        this.r = robot;
    }

    @Override
    public void initialize() {
        setState(0);
    }

    @Override
    public void execute() {
        double dist = r.getShootTarget().distanceFrom(r.f.getPose());
        r.s.forDistance(dist);
        
        switch (st) {
            case 0:
                r.s.down();
                r.i.spinIn();
                r.s.on();
                setState(1);
                break;
            case 1:
                if (r.s.atTarget() && t.getElapsedTime() > 250) {
                    setState(2);
                }
                break;
            case 2:
                if (t.getElapsedTimeSeconds() > 1.75) {
                    r.s.down();
                    setState(3);
                } else if (t.getElapsedTimeSeconds() > 1.5)
                    r.s.up();
                else if (t.getElapsedTimeSeconds() > 1.25)
                    r.s.down();
                else if (t.getElapsedTimeSeconds() > 1)
                    r.s.up();
                else if (t.getElapsedTimeSeconds() > .75)
                    r.s.down();
                else if (t.getElapsedTimeSeconds() > .5)
                    r.s.up();
                else if (t.getElapsedTimeSeconds() > 0.25)
                    r.s.down();
                else if (t.getElapsedTimeSeconds() > 0)
                    r.s.up();
                break;
            case 3:
                if (t.getElapsedTime() > 250) {
                    r.s.off();
                    r.i.spinIdle();
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
