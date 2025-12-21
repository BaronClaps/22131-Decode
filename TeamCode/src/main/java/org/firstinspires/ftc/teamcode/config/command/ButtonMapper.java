package org.firstinspires.ftc.teamcode.config.command;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.Scheduler;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

public class ButtonMapper {
    private final HashMap<BooleanSwitch, ICommand> buttonMap;

    public ButtonMapper() {
        buttonMap = new HashMap<>();
    }

    public ButtonMapper put(BooleanSwitch b, ICommand c) {
        buttonMap.put(b, c);
        return this;
    }

    public ButtonMapper put(BooleanSupplier b, ICommand c) {
        return put(new BooleanSwitch(b), c);
    }

    public void update() {
        for (Map.Entry<BooleanSwitch, ICommand> e : buttonMap.entrySet()) {
            e.getKey().update();

            if (e.getKey().isTrue())
                Scheduler.getInstance().schedule(e.getValue());
        }
    }
}