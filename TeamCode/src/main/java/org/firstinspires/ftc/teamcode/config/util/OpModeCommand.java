package org.firstinspires.ftc.teamcode.config.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.Subsystem;

public abstract class OpModeCommand extends OpMode {

    /**
     * Cancels all previous commands
     */
    public void reset() {
        CommandScheduler.getInstance().reset();
    }

    /**
     * Runs the {@link CommandScheduler} instance
     */
    public void run() {
        CommandScheduler.getInstance().run();
    }

    /**
     * Schedules objects to the scheduler
     */
    public void schedule(Command... commands) {
        CommandScheduler.getInstance().schedule(commands);
    }

    /**
     * Registers objects to the scheduler
     */
    public void register(Subsystem... subsystems) {
        CommandScheduler.getInstance().registerSubsystem(subsystems);
    }

    @Override
    public void init() {
        initialize();
    }

    @Override
    public void loop() {
        run();
    }

    public void stop() {
        reset();
    }

    public abstract void initialize();

}
