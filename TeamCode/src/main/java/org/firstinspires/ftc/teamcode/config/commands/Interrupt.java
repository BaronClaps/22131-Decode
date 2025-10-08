package org.firstinspires.ftc.teamcode.config.commands;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;

import java.util.function.BooleanSupplier;

public class Interrupt extends CommandBase {
    private Command command;
    private BooleanSupplier condition;
    private boolean done;

    public Interrupt(Command command, BooleanSupplier condition) {
        this.command = command;
        this.condition = condition;
        this.done = false;
    }

    @Override
    public void initialize() {
        this.command.initialize();
    }

    @Override
    public void execute() {
        this.command.execute();
        if (this.condition.getAsBoolean()) {
            this.command.end(false);
            this.done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return (this.done || this.command.isFinished());
    }
}