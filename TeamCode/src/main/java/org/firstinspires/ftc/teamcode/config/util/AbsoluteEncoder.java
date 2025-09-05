package org.firstinspires.ftc.teamcode.config.util;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class AbsoluteEncoder {
    public static double DEFAULT_RANGE = 3.24; // entropy encoders go up to 3.24 apparently
    public static boolean VALUE_REJECTION = false;
    private final AnalogInput encoder;
    private double offset, analogRange;
    private boolean inverted;

    /**
     * Make a regular encoder with the default range of DEFAULT_RANGE
     */
    public AbsoluteEncoder(AnalogInput enc){
        this(enc, DEFAULT_RANGE);
    }

    /**
     * Make a regular encoder with a custom range
     *
     * @param enc the encoder
     * @param aRange the range of the encoder
     */
    public AbsoluteEncoder(AnalogInput enc, double aRange){
        encoder = enc;
        analogRange = aRange;
        offset = 0;
        inverted = false;
    }

    /**
     * Sets the zero of the encoder
     *
     * @param off voltage to set the zero to
     *
     * @return this
     */
    public AbsoluteEncoder zero(double off){
        offset = off;
        return this;
    }
    /**
     * Sets the encoder to be inverted
     *
     * @param invert true to invert the encoder
     */
    public AbsoluteEncoder setInverted(boolean invert){
        inverted = invert;
        return this;
    }

    /**
     * Gets the direction (if its inverted)
     *
     * @returns true if the encoder is inverted
     */
    public boolean getDirection() {
        return inverted;
    }

    private double pastPosition = 1;

    /**
     * Gets the current position with angle normalization
     *
     * @returns the current position of the encoder in radians
     */
    public double getCurrentPosition() {
        double pos = norm((!inverted ? 1 - getVoltage() / analogRange : getVoltage() / analogRange) * Math.PI*2 - offset);
        //checks for crazy values when the encoder is close to zero
        if(!VALUE_REJECTION || Math.abs(normDelta(pastPosition)) > 0.1 || Math.abs(normDelta(pos)) < 1) pastPosition = pos;

        return pastPosition;
//        return MathUtils.round(pastPosition, 3);
    }

    /**
     * Returns this encoder as an AnalogInput
     *
     * @return analoginput of the encoder
     */
    public AnalogInput getEncoder() {
        return encoder;
    }

    /**
     * Returns the current voltage of the encoder
     *
     * @return voltage of the encoder
     */
    public double getVoltage(){
        return encoder.getVoltage();
    }

    private static double TAU = Math.PI * 2;

    /**
     * Returns [angle] clamped to `[0, 2pi]`.
     *
     * @param angle angle measure in radians
     * @return equivalent angle in [0, 2pi]
     */
    public static double norm(double angle) {
        double modifiedAngle = angle % TAU;

        modifiedAngle = (modifiedAngle + TAU) % TAU;

        return modifiedAngle;
    }

    /**
     * Returns [angleDelta] clamped to `[-pi, pi]`.
     *
     * @param angleDelta angle delta in radians
     */
    public static double normDelta(double angleDelta) {
        double modifiedAngleDelta = norm(angleDelta);

        if (modifiedAngleDelta > Math.PI) {
            modifiedAngleDelta -= TAU;
        }

        return modifiedAngleDelta;
    }
}
