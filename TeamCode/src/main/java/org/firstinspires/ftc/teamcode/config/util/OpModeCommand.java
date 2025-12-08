package org.firstinspires.ftc.teamcode.config.util;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class OpModeCommand extends OpMode {

    /**
     * Cancels all previous commands
     */
    public void reset() {
        Scheduler.getInstance().reset();
    }

    /**
     * Schedules objects to the scheduler
     */
    public void schedule(ICommand... commands) {
        Scheduler.getInstance().schedule(commands);
    }

    @Override
    public void init() {
    }

    @Override
    public void loop() {
        Scheduler.getInstance().execute();
    }

    public void stop() {
        reset();
    }
}
