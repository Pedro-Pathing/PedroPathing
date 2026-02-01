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

    public CoaxialPod(HardwareMap hardwareMap, String motorName, String servoName,
            String turnEncoderName, PIDFCoefficients turnPIDFCoefficients,
            DcMotorSimple.Direction driveDirection, CRServo.Direction servoDirection,
            double angleOffsetRad, Pose podOffset, double analogMinVoltage, double analogMaxVoltage,
            boolean encoderReversed) {

        this.driveMotor = hardwareMap.get(DcMotorEx.class, motorName);
        this.turnServo = hardwareMap.get(CRServo.class, servoName);
        this.turnEncoder = hardwareMap.get(AnalogInput.class, turnEncoderName);

        this.servoLabel = turnServo.getConnectionInfo(); // Best guess for label without explicit
                                                         // name

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

    @Override
    public Pose getOffset() {
        return offset;
    }

    @Override
    public double getAngle() {
        return getAngleAfterOffsetRad();
    }

    public void setServoPower(double power) {
        turnServo.setPower(power);
    }

    public void setMotorPower(double power) {
        driveMotor.setPower(power);
    }

    @Override
    public void setToFloat() {
        setMotorToFloat();
    }

    @Override
    public void setToBreak() {
        setMotorToBreak();
    }

    public void setMotorToFloat() {
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setMotorToBreak() {
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public boolean isEncoderReversed() {
        return encoderReversed;
    }

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
     * Commands pod to a wheel heading (radians) with a drive power [0, 1] This implementation pulls
     * caching thresholds and feedforward from internal state.
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
            turnServo.setPower(0);
        } else if (Math.abs(turnPower - turnServo.getPower()) > servoCachingThreshold) {
            turnServo.setPower(turnPower);
        }

        if (Math.abs(drivePower - driveMotor.getPower()) > motorCachingThreshold)
            driveMotor.setPower(drivePower);
    }

    public double getAngleAfterOffsetRad() {
        return getRawAngleRad() - angleOffsetRad;
    }

    public double getRawAngleRad() {
        double v = turnEncoder.getVoltage();
        double range = analogMaxVoltage - analogMinVoltage;
        if (range == 0)
            return 0;
        double normalized = (v - analogMinVoltage) / range;
        normalized = MathFunctions.clamp(normalized, 0, 1);
        return normalized * (2.0 * Math.PI);
    }

    public double getOffsetAngleRad() {
        double rad = getRawAngleRad() - angleOffsetRad;
        return MathFunctions.normalizeAngle(rad);
    }

    public void setMotorCachingThreshold(double motorCachingThreshold) {
        this.motorCachingThreshold = motorCachingThreshold;
    }

    public void setServoCachingThreshold(double servoCachingThreshold) {
        this.servoCachingThreshold = servoCachingThreshold;
    }

    @Override
    public String debugString() {
        return servoLabel + "{" + "current Angle(rad)=" + getRawAngleRad() + ", servo Power="
                + turnServo.getPower() + ", drive Power=" + driveMotor.getPower() + " }";
    }
}
