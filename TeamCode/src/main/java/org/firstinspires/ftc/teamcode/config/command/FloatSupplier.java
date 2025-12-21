package org.firstinspires.ftc.teamcode.config.command;

import java.util.function.Supplier;

public class FloatSupplier {
    private final Supplier<Float> floatSupplier;
    private float lastVal = 0;

    public FloatSupplier(Supplier<Float> floatSupplier) {
        this.floatSupplier = floatSupplier;
    }

    public float getVal() {
        return lastVal;
    }

    public void update() {
        lastVal = floatSupplier.get();
    }

    public BooleanSwitch clamped(float thresholdMin, float thresholdMax) {
        return new BooleanSwitch(() -> getVal() > thresholdMin && getVal() < thresholdMax).setUpdateFunction(this::update);
    }

    public BooleanSwitch greaterThan(float thresholdMin) {
        return new BooleanSwitch(() -> getVal() > thresholdMin).setUpdateFunction(this::update);
    }

    public BooleanSwitch lessThan(float thresholdMax) {
        return new BooleanSwitch(() -> getVal() < thresholdMax).setUpdateFunction(this::update);
    }
}