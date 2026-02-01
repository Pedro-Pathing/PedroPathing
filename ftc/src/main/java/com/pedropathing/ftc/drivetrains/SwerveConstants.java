package com.pedropathing.ftc.drivetrains;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Constants for swerve drive configuration.
 *
 * @author Kabir Goyal - 365 MOE
 * @author Baron Henderson
 */
public class SwerveConstants {

    // Velocity units should match your odometry/follower configuration.
    public double xVelocity = 80.0;
    public double yVelocity = 80.0;

    public boolean useBrakeModeInTeleOp = false;
    public double maxPower = 1.0;
    public boolean useVoltageCompensation = false;
    public double nominalVoltage = 12.0;
    public double staticFrictionCoefficient = 0.1;
    // Input deadband for translational/rotational joystick values.
    public double epsilon = 0.05;

    enum ZeroPowerBehavior {
        RESIST_MOVEMENT,
        IGNORE_ANGLE_CHANGES
    }

    public ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior.IGNORE_ANGLE_CHANGES;

    public SwerveConstants() {
        defaults();
    }

    /**
     * @param velocity the max speed in ANY direction because swerve can do that :)
     * @return this constants instance
     */
    public SwerveConstants velocity(double velocity) {
        this.xVelocity = velocity;
        this.yVelocity = velocity;
        return this;
    }

    /**
     * @param xVelocity maximum x velocity
     * @return this constants instance
     */
    public SwerveConstants xVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
        return this;
    }

    /**
     * @param yVelocity maximum y velocity
     * @return this constants instance
     */
    public SwerveConstants yVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
        return this;
    }

    /**
     * @param useBrakeModeInTeleOp true to enable brake mode in teleop
     * @return this constants instance
     */
    public SwerveConstants useBrakeModeInTeleOp(boolean useBrakeModeInTeleOp) {
        this.useBrakeModeInTeleOp = useBrakeModeInTeleOp;
        return this;
    }

    /**
     * @param maxPower maximum motor power
     * @return this constants instance
     */
    public SwerveConstants maxPower(double maxPower) {
        this.maxPower = maxPower;
        return this;
    }

    /**
     * @param useVoltageCompensation true to enable voltage compensation (not recommended)
     * @return this constants instance
     */
    public SwerveConstants useVoltageCompensation(boolean useVoltageCompensation) {
        this.useVoltageCompensation = useVoltageCompensation;
        return this;
    }

    /**
     * @param nominalVoltage nominal battery voltage used for compensation
     * @return this constants instance
     */
    public SwerveConstants nominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    /**
     * @param staticFrictionCoefficient static friction coefficient
     * @return this constants instance
     */
    public SwerveConstants staticFrictionCoefficient(double staticFrictionCoefficient) {
        this.staticFrictionCoefficient = staticFrictionCoefficient;
        return this;
    }

    /**
     * @param epsilon input deadband threshold
     * @return this constants instance
     */
    public SwerveConstants epsilon(double epsilon) {
        this.epsilon = epsilon;
        return this;
    }

    /**
     * @param zeroPowerBehavior behavior used at zero power
     * @return this constants instance
     */
    public SwerveConstants zeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        this.zeroPowerBehavior = zeroPowerBehavior;
        return this;
    }

    /**
     * @return maximum of x and y velocity
     */
    public double getVelocity() {
        return Math.max(getXVelocity(), getYVelocity());
    }

    /**
     * @return maximum x velocity
     */
    public double getXVelocity() {
        return xVelocity;
    }

    /**
     * @return maximum y velocity
     */
    public double getYVelocity() {
        return yVelocity;
    }

    /**
     * @return true if brake mode is enabled in teleop
     */
    public boolean getUseBrakeModeInTeleOp() {
        return useBrakeModeInTeleOp;
    }

    /**
     * @return maximum motor power
     */
    public double getMaxPower() {
        return maxPower;
    }

    /**
     * @return true if voltage compensation is enabled
     */
    public boolean getUseVoltageCompensation() {
        return useVoltageCompensation;
    }

    /**
     * @return nominal battery voltage used for compensation
     */
    public double getNominalVoltage() {
        return nominalVoltage;
    }

    /**
     * @return static friction coefficient
     */
    public double getStaticFrictionCoefficient() {
        return staticFrictionCoefficient;
    }

    /**
     * @return input deadband threshold
     */
    public double getEpsilon() {
        return epsilon;
    }

    /**
     * @return zero power behavior
     */
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return zeroPowerBehavior;
    }

    /**
     * @param velocity maximum speed applied to x and y
     */
    public void setVelocity(double velocity) {
        this.xVelocity = velocity;
        this.yVelocity = velocity;
    }

    /**
     * @param xVelocity maximum x velocity
     */
    public void setXVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
    }

    /**
     * @param yVelocity maximum y velocity
     */
    public void setYVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
    }

    /**
     * @param useBrakeModeInTeleOp true to enable brake mode in teleop
     */
    public void setUseBrakeModeInTeleOp(boolean useBrakeModeInTeleOp) {
        this.useBrakeModeInTeleOp = useBrakeModeInTeleOp;
    }

    /**
     * @param maxPower maximum motor power
     */
    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    /**
     * @param useVoltageCompensation true to enable voltage compensation (not recommended)
     */
    public void setUseVoltageCompensation(boolean useVoltageCompensation) {
        this.useVoltageCompensation = useVoltageCompensation;
    }

    /**
     * @param nominalVoltage nominal battery voltage used for compensation
     */
    public void setNominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
    }

    /**
     * @param staticFrictionCoefficient static friction coefficient
     */
    public void setStaticFrictionCoefficient(double staticFrictionCoefficient) {
        this.staticFrictionCoefficient = staticFrictionCoefficient;
    }

    /**
     * @param epsilon input deadband threshold
     */
    public void setEpsilon(double epsilon) {
        this.epsilon = epsilon;
    }

    /**
     * @param zeroPowerBehavior behavior used at zero power
     */
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        this.zeroPowerBehavior = zeroPowerBehavior;
    }

    /**
     * Resets all values to defaults.
     */
    public void defaults() {
        xVelocity = 80.0;
        yVelocity = 80.0;
        useBrakeModeInTeleOp = false;
        maxPower = 1.0;
        useVoltageCompensation = false;
        nominalVoltage = 12.0;
        staticFrictionCoefficient = 0.1;
        epsilon = 0.05;
        zeroPowerBehavior = ZeroPowerBehavior.IGNORE_ANGLE_CHANGES;
    }
}