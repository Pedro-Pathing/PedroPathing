package com.pedropathing.revhub.localizers;

import com.pedropathing.localization.EncoderPositionSource;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.function.IntSupplier;

/**
 * This is the Encoder class. This tracks encoder ticks from either a {@link DcMotorEx} or an
 * injected {@link EncoderPositionSource}. It stores previous and current positions and reports
 * a direction-aware delta.
 *
 * <p>Reading the current position, applying Pedro's multiplier, establishing a software zero,
 * and physically resetting an FTC motor encoder are separate operations:
 * <ul>
 *     <li>Motor-backed encoders keep the historical {@link DcMotor.RunMode#STOP_AND_RESET_ENCODER}
 *     behavior on {@link #reset()}.</li>
 *     <li>Injected sources rebase a software zero from the latest supplied value. They are never
 *     physically reset, because the same ticks may be shared with other consumers.</li>
 * </ul>
 *
 * <p>Injected values are raw hardware-sign ticks. Pedro applies {@link #setDirection(double)} to
 * deltas. Motor-backed encoders also multiply by {@link DcMotorSimple.Direction}, matching the
 * previous HardwareMap path. Injected sources do not consult a motor direction.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author The Allsparks - 36117
 * @version 1.1, 9/17/2026
 */
public class Encoder {
    private final EncoderPositionSource positionSource;
    private final DcMotorEx motor;
    private double previousPosition;
    private double currentPosition;
    private double multiplier;
    private int softwareZeroTicks;

    public final static double FORWARD = 1, REVERSE = -1;

    /**
     * This creates a new Encoder from a DcMotorEx. Construction physically resets the motor
     * encoder, matching historical Pedro behavior.
     *
     * @param setMotor the motor this will be tracking
     */
    public Encoder(DcMotorEx setMotor) {
        motor = setMotor;
        positionSource = setMotor::getCurrentPosition;
        multiplier = FORWARD;
        reset();
    }

    /**
     * This creates a new Encoder from an injected position source. Construction establishes a
     * software zero from the current source value and does not attempt a physical encoder reset.
     *
     * @param positionSource raw hardware-sign ticks; Pedro applies {@link #setDirection(double)}
     */
    public Encoder(EncoderPositionSource positionSource) {
        motor = null;
        this.positionSource = positionSource;
        multiplier = FORWARD;
        rebaseToCurrentPosition();
    }

    /**
     * This creates a new Encoder from an {@link IntSupplier} of raw ticks.
     *
     * @param positionTicks raw hardware-sign ticks; Pedro applies {@link #setDirection(double)}
     * @return encoder that rebases in software on reset
     */
    public static Encoder from(IntSupplier positionTicks) {
        return new Encoder(positionTicks::getAsInt);
    }

    /**
     * This sets the direction/multiplier of the Encoder. Setting 1 or -1 will make the Encoder track
     * forward or in reverse, respectively. Any multiple of either one will scale the Encoder's output
     * by that amount.
     *
     * @param setMultiplier the multiplier/direction to set
     */
    public void setDirection(double setMultiplier) {
        multiplier = setMultiplier;
    }

    /**
     * This resets the Encoder's tracked previous and current position.
     *
     * <p>Motor-backed encoders physically reset the FTC motor, then snapshot that hardware
     * position twice, matching historical behavior. Injected sources rebase a software zero
     * from one source read and never request a physical reset.
     */
    public void reset() {
        if (motor != null) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            previousPosition = motor.getCurrentPosition();
            currentPosition = motor.getCurrentPosition();
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            softwareZeroTicks = 0;
            return;
        }
        rebaseToCurrentPosition();
    }

    /**
     * Establishes a software zero from the latest supplied value without physically resetting
     * any hardware encoder.
     */
    public void rebaseToCurrentPosition() {
        softwareZeroTicks = positionSource.positionTicks();
        previousPosition = 0;
        currentPosition = 0;
    }

    /**
     * This updates the Encoder's tracked current position and previous position. The position
     * source is read exactly once.
     */
    public void update() {
        previousPosition = currentPosition;
        currentPosition = positionSource.positionTicks() - softwareZeroTicks;
    }

    /**
     * This returns the multiplier/direction of the Encoder. Motor-backed encoders also include
     * {@link DcMotorSimple.Direction}. Injected sources use only Pedro's configured multiplier.
     *
     * @return returns the multiplier
     */
    public double getMultiplier() {
        double motorDirection = 1;
        if (motor != null) {
            motorDirection = motor.getDirection() == DcMotorSimple.Direction.FORWARD ? 1 : -1;
        }
        return multiplier * motorDirection;
    }

    /**
     * This returns the change in position from the previous position to the current position. One
     * important thing to note is that this encoder does not track velocity, only change in position.
     * This is because I am using a pose exponential method of localization, which doesn't need the
     * velocity of the encoders. Velocity of the robot is calculated in the localizer using an elapsed
     * time timer there.
     *
     * @return returns the change in position of the Encoder
     */
    public double getDeltaPosition() {
        return getMultiplier() * (currentPosition - previousPosition);
    }
}
