package org.firstinspires.ftc.teamcode.config.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.config.Robot;

public class IntakeIn extends Command {
    private final Robot r;
    private int st = 0;
    private Timer t = new Timer();

    public IntakeIn(Robot robot) {
        this.r = robot;
    }

    @Override
    public void start() {
        setState(0);
    }

    @Override
    public void execute() {
        switch (st) {
            case 0:
                r.s.down();
                r.i.spinIn();
                setState(1);
                break;
            case 1:
                if (t.getElapsedTime() > 500) {
                    setState(-1);
                }
                break;
        }
    }


    @Override
    public boolean done() {
        return st == -1;
    }

    public void setState(int x) {
        st = x;
        t.resetTimer();
    }

}
