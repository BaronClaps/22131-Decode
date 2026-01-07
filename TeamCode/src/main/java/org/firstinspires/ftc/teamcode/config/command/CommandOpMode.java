package org.firstinspires.ftc.teamcode.config.command;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class CommandOpMode extends OpMode {

    /**
     * Cancels all previous commands
     */
    public void reset() {
        Scheduler.reset();
    }

    /**
     * Schedules objects to the scheduler
     */
    public void schedule(Command... commands) {
        Scheduler.schedule(commands);
    }

    @Override
    public void init() {
    }

    @Override
    public void loop() {
        Scheduler.execute();
    }

    public void stop() {
        reset();
    }
}
