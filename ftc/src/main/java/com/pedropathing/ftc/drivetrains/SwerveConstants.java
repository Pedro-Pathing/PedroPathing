package com.pedropathing.ftc.drivetrains;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Constants for swerve drive configuration
 *
 * @author Kabir Goyal - 365 MOE
 * @author Baron Henderson
 */
public class SwerveConstants {

    public double xVelocity = 80.0;
    public double yVelocity = 80.0;

    public boolean useBrakeModeInTeleOp = false;
    public double maxPower = 1.0;
    public boolean useVoltageCompensation = false;
    public double nominalVoltage = 12.0;
    public double staticFrictionCoefficient = 0.1;
    public double epsilon = 0.001;

    public String leftFrontMotorName = "frontLeftDrive";
    public String leftFrontServoName = "frontLeftTurnServo";
    public String leftFrontEncoderName = "frontLeftTurnEncoder";

    public String rightFrontMotorName = "frontRightDrive";
    public String rightFrontServoName = "frontRightTurnServo";
    public String rightFrontEncoderName = "frontRightTurnEncoder";

    public String leftRearMotorName = "backLeftDrive";
    public String leftRearServoName = "backLeftTurnServo";
    public String leftRearEncoderName = "backLeftTurnEncoder";

    public String rightRearMotorName = "backRightDrive";
    public String rightRearServoName = "backRightTurnServo";
    public String rightRearEncoderName = "backRightTurnEncoder";

    // TODO: Change PID coefficients based on your config
    public PIDFCoefficients leftFrontTurnPID = new PIDFCoefficients(0.005, 0.0, 0.001, 0.0);
    public PIDFCoefficients rightFrontTurnPID = new PIDFCoefficients(0.005, 0.0, 0.001, 0.0);
    public PIDFCoefficients leftRearTurnPID = new PIDFCoefficients(0.005, 0.0, 0.001, 0.0);
    public PIDFCoefficients rightRearTurnPID = new PIDFCoefficients(0.005, 0.0, 0.001, 0.0);

    // TODO: Reverse motors if needed
    public DcMotorEx.Direction leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
    public DcMotorEx.Direction rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
    public DcMotorEx.Direction leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
    public DcMotorEx.Direction rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

    // TODO: Reverse servos if needed
    public CRServo.Direction leftFrontServoDirection = CRServo.Direction.FORWARD;
    public CRServo.Direction rightFrontServoDirection = CRServo.Direction.FORWARD;
    public CRServo.Direction leftRearServoDirection = CRServo.Direction.FORWARD;
    public CRServo.Direction rightRearServoDirection = CRServo.Direction.FORWARD;

    // TODO: These are the reported angle of each pod when facing forward, in
    // degrees
    public double leftFrontPodAngleOffsetDeg = 0.0;
    public double rightFrontPodAngleOffsetDeg = 0.0;
    public double leftRearPodAngleOffsetDeg = 0.0;
    public double rightRearPodAngleOffsetDeg = 0.0;

    // TODO: distance from the center of the robot to each pod
    // if you're swerve is square, you can leave these be
    // hopefully these should be +- the same x, y, but just in case
    // units don't matter, they'll be normalized
    // positive x is right, positive y is forward, as with joysticks
    public double[] leftFrontPodXYOffsets = new double[] { -1, 1 };
    public double[] rightFrontPodXYOffsets = new double[] { 1, 1 };
    public double[] leftRearPodXYOffsets = new double[] { -1, -1 };
    public double[] rightRearPodXYOffsets = new double[] { 1, -1 };

    public double leftFrontReferenceVoltage = 3.3;
    public double rightFrontReferenceVoltage = 3.3;
    public double leftRearReferenceVoltage = 3.3;
    public double rightRearReferenceVoltage = 3.3;

    // use to reverse your encoder if positive is counterclockwise
    public boolean leftFrontEncoderReversed = false;
    public boolean rightFrontEncoderReversed = false;
    public boolean leftRearEncoderReversed = false;
    public boolean rightRearEncoderReversed = false;


    enum ZeroPowerBehavior {
        RESIST_MOVEMENT,
        IGNORE_ANGLE_CHANGES
    }

    public ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior.RESIST_MOVEMENT;

    public SwerveConstants() {
        defaults();
    }

    /**
     * @param velocity the max speed in ANY direction because swerve can do that :)
     * @return
     */
    public SwerveConstants velocity(double velocity) {
        this.xVelocity = velocity;
        this.yVelocity = velocity;
        return this;
    }

    public SwerveConstants xVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
        return this;
    }

    public SwerveConstants yVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
        return this;
    }

    public SwerveConstants useBrakeModeInTeleOp(boolean useBrakeModeInTeleOp) {
        this.useBrakeModeInTeleOp = useBrakeModeInTeleOp;
        return this;
    }

    public SwerveConstants maxPower(double maxPower) {
        this.maxPower = maxPower;
        return this;
    }

    public SwerveConstants useVoltageCompensation(boolean useVoltageCompensation) {
        this.useVoltageCompensation = useVoltageCompensation;
        return this;
    }

    public SwerveConstants nominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public SwerveConstants staticFrictionCoefficient(double staticFrictionCoefficient) {
        this.staticFrictionCoefficient = staticFrictionCoefficient;
        return this;
    }

    public SwerveConstants epsilon(double epsilon) {
        this.epsilon = epsilon;
        return this;
    }

    public SwerveConstants zeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        this.zeroPowerBehavior = zeroPowerBehavior;
        return this;
    }

    public double getVelocity() {
        return Math.max(getXVelocity(), getYVelocity());
    }

    public double getXVelocity() {
        return xVelocity;
    }

    public double getYVelocity() {
        return yVelocity;
    }

    public boolean getUseBrakeModeInTeleOp() {
        return useBrakeModeInTeleOp;
    }

    public double getMaxPower() {
        return maxPower;
    }

    public boolean getUseVoltageCompensation() {
        return useVoltageCompensation;
    }

    public double getNominalVoltage() {
        return nominalVoltage;
    }

    public double getStaticFrictionCoefficient() {
        return staticFrictionCoefficient;
    }

    public double getEpsilon() {
        return epsilon;
    }

    public ZeroPowerBehavior getZeroPowerBehavior() {
        return zeroPowerBehavior;
    }

    public void setVelocity(double velocity) {
        this.xVelocity = velocity;
        this.yVelocity = velocity;
    }

    public void setXVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
    }

    public void setYVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
    }

    public void setUseBrakeModeInTeleOp(boolean useBrakeModeInTeleOp) {
        this.useBrakeModeInTeleOp = useBrakeModeInTeleOp;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public void setUseVoltageCompensation(boolean useVoltageCompensation) {
        this.useVoltageCompensation = useVoltageCompensation;
    }

    public void setNominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
    }

    public void setStaticFrictionCoefficient(double staticFrictionCoefficient) {
        this.staticFrictionCoefficient = staticFrictionCoefficient;
    }

    public void setEpsilon(double epsilon) {
        this.epsilon = epsilon;
    }

    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        this.zeroPowerBehavior = zeroPowerBehavior;
    }

    public void defaults() {
        xVelocity = 80.0;
        yVelocity = 80.0;
        useBrakeModeInTeleOp = false;
        maxPower = 1.0;
        useVoltageCompensation = false;
        nominalVoltage = 12.0;
        staticFrictionCoefficient = 0.1;
        epsilon = 0.001;
        zeroPowerBehavior = ZeroPowerBehavior.RESIST_MOVEMENT;
    }
}