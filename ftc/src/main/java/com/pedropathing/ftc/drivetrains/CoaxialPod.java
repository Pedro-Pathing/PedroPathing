package com.pedropathing.ftc.drivetrains;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.*;

/**
 * CoaxialPod is a hardware-backed implementation of the core `SwervePod` interface. It owns the
 * drive motor, continuous rotation servo (turn), analog encoder and the PIDF controller used to
 * control pod rotation.
 *
 * @author Kabir Goyal
 * @author Baron Henderson
 */
public class CoaxialPod implements SwervePod {
    private final AnalogInput turnEncoder; // for rotation of servo
    private final CRServo turnServo;
    private final DcMotorEx driveMotor;

    private final PIDFController turnPID;
    private final Pose offset;

    // Angle offset in radians applied to raw encoder angle
    private final double angleOffsetRad;

    private final String servoLabel;

    // analog encoder voltage range (min -> max), e.g. 0.0 -> 3.3 V
    private final double analogMinVoltage;
    private final double analogMaxVoltage;

    private final boolean encoderReversed;

    private double motorCachingThreshold = 0.01;
    private double servoCachingThreshold = 0.01;

    private double lastDrivePower = 0;
    private double lastTurnPower = 0;

    /**
     * @param motorName drive motor name
     * @param servoName turn servo name
     * @param turnEncoderName analog encoder name
     * @param turnPIDFCoefficients PIDF coefficients for servo control
     * @param driveDirection drive motor direction
     * @param servoDirection turn servo direction
     * @param angleOffsetRad offset applied to raw encoder angle, in radians. This is the raw angle
     *                       in radians when the wheel is facing forward.
     * @param podOffset pod position offset from robot center, using the same axes as odometry pods
     * @param analogMinVoltage minimum encoder voltage (e.g. 0.0)
     * @param analogMaxVoltage maximum encoder voltage (e.g. 3.3)
     * @param encoderReversed true if encoder increases CCW (top-down)
     */
    public CoaxialPod(HardwareMap hardwareMap, String motorName, String servoName,
            String turnEncoderName, PIDFCoefficients turnPIDFCoefficients,
            DcMotorSimple.Direction driveDirection, CRServo.Direction servoDirection,
            double angleOffsetRad, Pose podOffset, double analogMinVoltage, double analogMaxVoltage,
            boolean encoderReversed) {

        this.driveMotor = hardwareMap.get(DcMotorEx.class, motorName);
        this.turnServo = hardwareMap.get(CRServo.class, servoName);
        this.turnEncoder = hardwareMap.get(AnalogInput.class, turnEncoderName);

        this.servoLabel = servoName;

        this.turnPID = new PIDFController(turnPIDFCoefficients);
        this.angleOffsetRad = angleOffsetRad;

        setMotorToFloat();

        driveMotor.setDirection(driveDirection);
        turnServo.setDirection(servoDirection);

        this.analogMinVoltage = analogMinVoltage;
        this.analogMaxVoltage = analogMaxVoltage;
        this.encoderReversed = encoderReversed;

        this.offset = podOffset;

        turnServo.setPower(0);
    }

    /**
     * Returns the pod's offset from robot center.
     *
     * @return offset as a Pose
     */
    @Override
    public Pose getOffset() {
        return offset;
    }

    /**
     * Returns the current pod heading after applying the configured offset, in radians.
     *
     * @return heading in radians
     */
    @Override
    public double getAngle() {
        return getAngleAfterOffsetRad();
    }

    /**
     * Sets turn servo power in [-1, 1].
     *
     * @param power turn servo power
     */
    public void setServoPower(double power) {
        lastTurnPower = power;
        turnServo.setPower(power);
    }

    /**
     * Sets drive motor power in [-1, 1].
     *
     * @param power drive motor power
     */
    public void setMotorPower(double power) {
        lastDrivePower = power;
        driveMotor.setPower(power);
    }

    /**
     * Sets drive motor zero power behavior to FLOAT.
     */
    @Override
    public void setToFloat() {
        setMotorToFloat();
    }

    /**
     * Sets drive motor zero power behavior to BRAKE.
     */
    @Override
    public void setToBreak() {
        setMotorToBreak();
    }

    /**
     * Sets drive motor zero power behavior to FLOAT.
     */
    public void setMotorToFloat() {
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Sets drive motor zero power behavior to BRAKE.
     */
    public void setMotorToBreak() {
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * @return encoder reversed status
     */
    public boolean isEncoderReversed() {
        return encoderReversed;
    }

    /**
     * Converts wheel-space theta (radians) to encoder-space theta.
     *
     * @param wheelTheta wheel-space heading in radians
     * @return encoder-space heading in radians
     */
    @Override
    public double adjustThetaForEncoder(double wheelTheta) {
        // wheelTheta is in radians. If encoder is reversed, use wheelTheta directly; otherwise invert.
        //if encoder is reversed, ccw (top down) is positive, if unreversed than cw is positive
        double t = encoderReversed ? wheelTheta : (2 * Math.PI - wheelTheta);
        // servo zero offset: +90 degrees -> +pi/2 radians
        t += Math.PI / 2.0;
        return MathFunctions.normalizeAngle(t);
    }

    /**
     * Commands pod to a wheel heading (radians) with a drive power in [-1, 1].
     *
     * @param targetAngleRad desired wheel heading in radians
     * @param drivePower drive power in [0, 1]
     * @param ignoreAngleChanges if true, turn servo power is set to 0 regardless of target angle
     */
    @Override
    public void move(double targetAngleRad, double drivePower, boolean ignoreAngleChanges) {
        // Convert hardware angle to radians and normalize
        double actualRad = getAngleAfterOffsetRad();
        actualRad = MathFunctions.normalizeAngle(actualRad);

        //if encoder is reversed, ccw (top down) is positive, if unreversed than cw is positive
        double desiredRad = encoderReversed ? targetAngleRad : (2 * Math.PI - targetAngleRad);
        desiredRad += Math.PI / 2.0;
        desiredRad = MathFunctions.normalizeAngle(desiredRad);

        // Shortest-path error in radians (signed)
        double mag = MathFunctions.getSmallestAngleDifference(actualRad, desiredRad);
        double dir = MathFunctions.getTurnDirection(actualRad, desiredRad);
        double signedRad = (mag == Math.PI) ? -Math.PI : mag * dir;

        // PID uses radians (tune PIDF for radian error)
        double errorRad = signedRad;

        // Minimize rotation: flip + invert drive if > 90°
        if (Math.abs(errorRad) > (Math.PI / 2.0)) {
            // add 180 degrees (pi radians)
            desiredRad = MathFunctions.normalizeAngle(desiredRad + Math.PI);
            drivePower = -drivePower;

            // recompute signed error
            mag = MathFunctions.getSmallestAngleDifference(actualRad, desiredRad);
            dir = MathFunctions.getTurnDirection(actualRad, desiredRad);
            signedRad = (mag == Math.PI) ? -Math.PI : mag * dir;
            errorRad = signedRad;
        }

        // Setpoint close to current so PID follows shortest path
        double setpointRad = actualRad + errorRad;

        if (Math.abs(errorRad) < (2.0 * Math.PI / 180.0)) {
            turnPID.updateFeedForwardInput(0);
        } else {
            turnPID.updateFeedForwardInput(MathFunctions.getTurnDirection(actualRad, desiredRad));
        }

        turnPID.updateError(setpointRad - actualRad);
        double turnPower = MathFunctions.clamp(turnPID.run(), -1.0, 1.0);

        // please don't change the next 5 lines took like 5 hours to figure ts out
        if (ignoreAngleChanges) {
            lastTurnPower = 0;
            turnServo.setPower(0);
        } else if (Math.abs(turnPower - lastTurnPower) > servoCachingThreshold || (turnPower == 0 && lastTurnPower != 0)) {
            lastTurnPower = turnPower;
            turnServo.setPower(turnPower);
        }

        if (Math.abs(drivePower - lastDrivePower) > motorCachingThreshold || (drivePower == 0 && lastDrivePower != 0)) {
            lastDrivePower = drivePower;
            driveMotor.setPower(drivePower);
        }
    }

    /**
     * Returns the current pod heading after applying the configured offset, in radians.
     *
     * @return heading in radians
     */
    public double getAngleAfterOffsetRad() {
        return getRawAngleRad() - angleOffsetRad;
    }

    /**
     * Returns the raw encoder angle in radians, in [0, 2pi].
     *
     * @return raw encoder angle in radians
     */
    public double getRawAngleRad() {
        double v = turnEncoder.getVoltage();
        double range = analogMaxVoltage - analogMinVoltage;
        if (range == 0)
            return 0;
        double normalized = (v - analogMinVoltage) / range;
        normalized = MathFunctions.clamp(normalized, 0, 1);
        return normalized * (2.0 * Math.PI);
    }

    /**
     * Returns the normalized raw angle after offset, in radians.
     *
     * @return normalized angle in radians
     */
    public double getOffsetAngleRad() {
        double rad = getRawAngleRad() - angleOffsetRad;
        return MathFunctions.normalizeAngle(rad);
    }

    /**
     * Sets the drive motor caching threshold for power updates.
     *
     * @param motorCachingThreshold minimum delta before applying power update
     */
    public void setMotorCachingThreshold(double motorCachingThreshold) {
        this.motorCachingThreshold = motorCachingThreshold;
    }

    /**
     * Sets the turn servo caching threshold for power updates.
     *
     * @param servoCachingThreshold minimum delta before applying power update
     */
    public void setServoCachingThreshold(double servoCachingThreshold) {
        this.servoCachingThreshold = servoCachingThreshold;
    }

    /**
     * @return debug string for pod state
     */
    @Override
    public String debugString() {
        double rawAngleRad = getRawAngleRad();
        double offsetAngleRad = getAngleAfterOffsetRad();
        return servoLabel + " {" + "\ncurrent raw angle (rad/deg) = " + rawAngleRad + " / " + Math.toDegrees(rawAngleRad)
                + "\ncurrent angle after offset (rad/deg) = " + offsetAngleRad + " / " + Math.toDegrees(offsetAngleRad)
                + "\nservo Power = " + turnServo.getPower()
                + "\ndrive Power = " + driveMotor.getPower()
                + "\n}";
    }
}
