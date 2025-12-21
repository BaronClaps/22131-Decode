package org.firstinspires.ftc.teamcode.config.command;

import java.util.function.BooleanSupplier;

public final class BooleanSwitch {
    private boolean state;
    private boolean previousState;
    private boolean toggled;
    private final BooleanSupplier decider;
    private Runnable updateFunction;

    public BooleanSwitch(BooleanSupplier decider) {
        this.decider = decider;
    }

    /** Advance the state by one tick */
    public void update() {
        if (updateFunction != null) updateFunction.run();
        previousState = state;
        state = decider.getAsBoolean();

        // toggle on rising edge
        if (isRisingEdge()) {
            toggled = !toggled;
        }
    }

    public boolean isTrue() {
        return state;
    }

    public boolean isRisingEdge() {
        return state && !previousState;
    }

    public boolean isFallingEdge() {
        return !state && previousState;
    }

    public boolean isToggled() {
        return toggled;
    }

    public BooleanSwitch risingEdge() {
        return new BooleanSwitch(this::isRisingEdge).setUpdateFunction(this::update);
    }

    public BooleanSwitch fallingEdge() {
        return new BooleanSwitch(this::isFallingEdge).setUpdateFunction(this::update);
    }

    public BooleanSwitch toggled() {
        return new BooleanSwitch(this::isToggled).setUpdateFunction(this::update);
    }

    public BooleanSwitch and(BooleanSwitch other) {
        return new BooleanSwitch(() -> this.state && other.state);
    }

    public BooleanSwitch and(BooleanSupplier other) {
        return and(new BooleanSwitch(other));
    }

    public BooleanSwitch or(BooleanSwitch other) {
        return new BooleanSwitch(() -> this.state || other.state);
    }

    public BooleanSwitch or(BooleanSupplier other) {
        return or(new BooleanSwitch(other));
    }

    public BooleanSwitch not() {
        return new BooleanSwitch(() -> !this.state);
    }

    public BooleanSwitch setUpdateFunction(Runnable r) {
        this.updateFunction = r;
        return this;
    }
}